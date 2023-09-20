#!/usr/bin/env python

import math
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from pymodbus.client import ModbusSerialClient as ModbusClient
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("twist_cmd_vel_topic",
                               "/zlac8015d/twist/cmd_vel")
        self.declare_parameter("cmd_vel_topic", "/zlac8015d/vel/cmd_vel")
        self.declare_parameter("cmd_rpm_topic", "/zlac8015d/vel/cmd_rpm")
        self.declare_parameter("cmd_deg_topic", "/zlac8015d/pos/cmd_deg")
        self.declare_parameter("cmd_dist_topic", "/zlac8015d/pos/cmd_dist")
        self.declare_parameter("control_mode", 3)
        self.declare_parameter("callback_timeout", 0.5)
        self.declare_parameter("wheels_base_width", 0.5668)
        self.declare_parameter("left_wheel_radius", 0.1015)
        self.declare_parameter("right_wheel_radius", 0.1015)
        self.declare_parameter("computation_left_wheel_radius", 0.1015)
        self.declare_parameter("computation_right_wheel_radius", 0.1015)
        self.declare_parameter("cpr", 16385)
        self.declare_parameter("deadband_rpm", 3)
        self.declare_parameter("max_left_rpm", 150)
        self.declare_parameter("max_right_rpm", 150)
        self.declare_parameter("publish_TF", True)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("TF_header_frame", "odom")
        self.declare_parameter("TF_child_frame", "base_footprint")
        self.declare_parameter("odom_header_frame", "odom")
        self.declare_parameter("odom_child_frame", "base_footprint")
        self.declare_parameter("set_accel_time_left", 200)
        self.declare_parameter("set_accel_time_right", 200)
        self.declare_parameter("set_decel_time_left", 200)
        self.declare_parameter("set_decel_time_right", 200)
        self.declare_parameter("debug", False)

        self.client = ModbusClient(
            port=self.get_parameter("port").get_parameter_value().string_value,
            baudrate=115200,
            timeout=1,
        )
        self.client.connect()
        self.ID = 1

        # -----Register Address-----
        # Common
        self.CONTROL_REG = 0x200E
        self.OPR_MODE = 0x200D
        self.L_ACL_TIME = 0x2080
        self.L_DCL_TIME = 0x2082

        # Speed RPM Control
        self.L_CMD_RPM = 0x2088

        # Position Control
        self.POS_CONTROL_TYPE = 0x200F
        self.L_MAX_RPM_POS = 0x208E
        self.L_CMD_REL_POS_HI = 0x208A
        self.L_FB_POS_HI = 0x20A7

        # -----Control CMDs (REG)-----
        self.EMER_STOP = 0x05
        self.ALRM_CLR = 0x06
        self.DOWN_TIME = 0x07
        self.ENABLE = 0x08
        self.POS_L_START = 0x11
        self.POS_R_START = 0x12

        # -----Operation Mode-----
        self.ASYNC = 0

        # -----Initialize Publisher-----
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.odom_msg = Odometry()

        # -----Initialize subscriber-----
        self.create_subscription(
            Twist,
            self.get_parameter("twist_cmd_vel_topic")
            .get_parameter_value()
            .string_value,
            self.twist_cmd_callback,
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter(
                "cmd_vel_topic").get_parameter_value().string_value,
            self.vel_cmd_callback,
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter(
                "cmd_rpm_topic").get_parameter_value().string_value,
            self.rpm_cmd_callback,
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter(
                "cmd_deg_topic").get_parameter_value().string_value,
            self.deg_cmd_callback,
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter(
                "cmd_dist_topic").get_parameter_value().string_value,
            self.dist_cmd_callback,
            1,
        )
        self.create_subscription(Bool, "/estop", self.estop_callback, 1)

        # -----Initialize Control Mode-----
        self.control_mode = (
            self.get_parameter(
                "control_mode").get_parameter_value().integer_value
        )
        if self.control_mode == 3:
            self.speed_mode_init()
        elif self.control_mode == 1:
            self.position_mode_init()

        # -----Initialize Variable-----
        self.callback_timeout = (
            self.get_parameter(
                "callback_timeout").get_parameter_value().double_value
        )
        self.wheels_base_width = (
            self.get_parameter(
                "wheels_base_width").get_parameter_value().double_value
        )
        self.left_wheel_radius = (
            self.get_parameter(
                "left_wheel_radius").get_parameter_value().double_value
        )
        self.right_wheel_radius = (
            self.get_parameter(
                "right_wheel_radius").get_parameter_value().double_value
        )
        self.computation_left_wheel_radius = (
            self.get_parameter("computation_left_wheel_radius")
            .get_parameter_value()
            .double_value
        )
        self.computation_right_wheel_radius = (
            self.get_parameter("computation_right_wheel_radius")
            .get_parameter_value()
            .double_value
        )
        self.cpr = self.get_parameter(
            "cpr").get_parameter_value().integer_value
        self.deadband_rpm = (
            self.get_parameter(
                "deadband_rpm").get_parameter_value().integer_value
        )
        self.left_rpm_lim = (
            self.get_parameter(
                "max_left_rpm").get_parameter_value().integer_value
        )
        self.right_rpm_lim = (
            self.get_parameter(
                "max_right_rpm").get_parameter_value().integer_value
        )
        self.publish_TF = (
            self.get_parameter("publish_TF").get_parameter_value().bool_value
        )
        self.publish_odom = (
            self.get_parameter("publish_odom").get_parameter_value().bool_value
        )
        self.TF_header_frame = (
            self.get_parameter(
                "TF_header_frame").get_parameter_value().string_value
        )
        self.TF_child_frame = (
            self.get_parameter(
                "TF_child_frame").get_parameter_value().string_value
        )
        self.odom_header_frame = (
            self.get_parameter(
                "odom_header_frame").get_parameter_value().string_value
        )
        self.odom_child_frame = (
            self.get_parameter(
                "odom_child_frame").get_parameter_value().string_value
        )
        self.debug = self.get_parameter(
            "debug").get_parameter_value().bool_value

        self.linear_vel_cmd = 0.0
        self.angular_vel_cmd = 0.0
        self.got_twist_cmd = False
        self.left_vel_cmd = 0.0
        self.right_vel_cmd = 0.0
        self.got_vel_cmd = False
        self.left_rpm_cmd = 0.0
        self.right_rpm_cmd = 0.0
        self.got_vel_rpm_cmd = False
        self.left_pos_deg_cmd = 0.0
        self.right_pos_deg_cmd = 0.0
        self.got_pos_deg_cmd = False
        self.left_pos_dist_cmd = 0.0
        self.right_pos_dist_cmd = 0.0
        self.got_pos_dist_cmd = False
        self.estop = False
        self.estop_reset_flag = False
        self.last_subscribed_time = 0.0
        self.period = 0.05
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.l_meter = 0.0
        self.r_meter = 0.0
        self.prev_l_meter = 0.0
        self.prev_r_meter = 0.0

        self.l_meter_init, self.r_meter_init = self.get_wheels_travelled()
        self.t = TransformStamped()
        self.br = tf2_ros.TransformBroadcaster(self)
        self.state_vector = np.zeros((3, 1))

        self.get_logger().info("Start ZLAC8015D motor driver node")
        self.get_logger().info("#########################")

    def speed_mode_init(self):
        # -----Disable Motor-----
        result = self.client.write_register(
            self.CONTROL_REG, self.DOWN_TIME, slave=self.ID
        )

        # -----Set Accel Time-----
        AL_ms = (
            self.get_parameter("set_accel_time_left")
            .get_parameter_value()
            .integer_value
        )
        AR_ms = (
            self.get_parameter("set_accel_time_right")
            .get_parameter_value()
            .integer_value
        )
        if AL_ms > 32767:
            AL_ms = 32767
        elif AL_ms < 0:
            AL_ms = 0

        if AR_ms > 32767:
            AR_ms = 32767
        elif AR_ms < 0:
            AR_ms = 0

        result = self.client.write_registers(
            self.L_ACL_TIME, [AL_ms, AR_ms], slave=self.ID
        )

        # -----Set Decel Time-----
        DL_ms = (
            self.get_parameter("set_decel_time_left")
            .get_parameter_value()
            .integer_value
        )
        DR_ms = (
            self.get_parameter("set_decel_time_right")
            .get_parameter_value()
            .integer_value
        )
        if DL_ms > 32767:
            DL_ms = 32767
        elif DL_ms < 0:
            DL_ms = 0

        if DR_ms > 32767:
            DR_ms = 32767
        elif DR_ms < 0:
            DR_ms = 0

        result = self.client.write_registers(
            self.L_DCL_TIME, [DL_ms, DR_ms], slave=self.ID
        )

        # -----Set Mode-----
        mode = 3
        result = self.client.write_register(self.OPR_MODE, mode, slave=self.ID)
        self.get_logger().info("Set mode as speed RPM control")

        # -----Enable Motor-----
        result = self.client.write_register(
            self.CONTROL_REG, self.ENABLE, slave=self.ID
        )

    def position_mode_init(self):
        # -----Disable Motor-----
        result = self.client.write_register(
            self.CONTROL_REG, self.DOWN_TIME, slave=self.ID
        )

        # -----Set Accel Time-----
        AL_ms = (
            self.get_parameter("set_accel_time_left")
            .get_parameter_value()
            .integer_value
        )
        AR_ms = (
            self.get_parameter("set_accel_time_right")
            .get_parameter_value()
            .integer_value
        )
        if AL_ms > 32767:
            AL_ms = 32767
        elif AL_ms < 0:
            AL_ms = 0

        if AR_ms > 32767:
            AR_ms = 32767
        elif AR_ms < 0:
            AR_ms = 0

        result = self.client.write_registers(
            self.L_ACL_TIME, [AL_ms, AR_ms], slave=self.ID
        )

        # -----Set Decel Time-----
        DL_ms = (
            self.get_parameter("set_decel_time_left")
            .get_parameter_value()
            .integer_value
        )
        DR_ms = (
            self.get_parameter("set_decel_time_right")
            .get_parameter_value()
            .integer_value
        )
        if DL_ms > 32767:
            DL_ms = 32767
        elif DL_ms < 0:
            DL_ms = 0

        if DR_ms > 32767:
            DR_ms = 32767
        elif DR_ms < 0:
            DR_ms = 0

        result = self.client.write_registers(
            self.L_DCL_TIME, [DL_ms, DR_ms], slave=self.ID
        )

        # -----Set Mode-----
        mode = 1
        result = self.client.write_register(self.OPR_MODE, mode, slave=self.ID)
        self.get_logger().info("Set mode as relative position control")

        # -----Set Position Async Control-----
        result = self.client.write_register(
            self.POS_CONTROL_TYPE, self.ASYNC, slave=self.ID
        )

        # -----Set Position Async Control-----
        max_L_rpm = (
            self.get_parameter(
                "max_left_rpm").get_parameter_value().integer_value
        )
        max_R_rpm = (
            self.get_parameter(
                "max_right_rpm").get_parameter_value().integer_value
        )
        if max_L_rpm > 1000:
            max_L_rpm = 1000
        elif max_L_rpm < 1:
            max_L_rpm = 1

        if max_R_rpm > 1000:
            max_R_rpm = 1000
        elif max_R_rpm < 1:
            max_R_rpm = 1

        result = self.client.write_registers(
            self.L_MAX_RPM_POS, [max_L_rpm, max_R_rpm], slave=self.ID
        )

        # -----Enable Motor-----
        result = self.client.write_register(
            self.CONTROL_REG, self.ENABLE, slave=self.ID
        )

    def twist_cmd_callback(self, msg):
        self.linear_vel_cmd = msg.linear.x
        self.angular_vel_cmd = msg.angular.z
        self.got_twist_cmd = True
        self.last_subscribed_time = time.perf_counter()

    def vel_cmd_callback(self, msg):
        self.left_vel_cmd = msg.data[0]
        self.right_vel_cmd = -msg.data[1]
        self.got_vel_cmd = True
        self.last_subscribed_time = time.perf_counter()

    def rpm_cmd_callback(self, msg):
        self.left_rpm_cmd = msg.data[0]
        self.right_rpm_cmd = -msg.data[1]
        self.got_vel_rpm_cmd = True
        self.last_subscribed_time = time.perf_counter()

    def deg_cmd_callback(self, msg):
        self.left_pos_deg_cmd = msg.data[0]
        self.right_pos_deg_cmd = -msg.data[1]
        self.got_pos_deg_cmd = True

    def dist_cmd_callback(self, msg):
        self.left_pos_dist_cmd = msg.data[0]
        self.right_pos_dist_cmd = msg.data[1]
        self.got_pos_dist_cmd = True

    def estop_callback(self, msg):
        self.estop = msg.data

    def twist_to_rpm(self, linear_vel, angular_vel):
        left_vel = linear_vel - self.wheels_base_width / 2 * angular_vel
        right_vel = linear_vel + self.wheels_base_width / 2 * angular_vel
        left_rpm, right_rpm = self.vel_to_rpm(left_vel, -right_vel)
        return left_rpm, right_rpm

    def vel_to_rpm(self, left_vel, right_vel):
        left_rpm = 60 * left_vel / (2 * np.pi * self.left_wheel_radius)
        right_rpm = 60 * right_vel / (2 * np.pi * self.right_wheel_radius)
        return left_rpm, right_rpm

    def dist_to_relative_angle(self, left_dist, right_dist):
        left_circumference = self.left_wheel_radius * 2 * np.pi
        right_circumference = self.right_wheel_radius * 2 * np.pi
        left_relative_deg = (left_dist * 360.0) / left_circumference
        right_relative_deg = (-right_dist * 360.0) / right_circumference
        return left_relative_deg, right_relative_deg

    def int16Dec_to_int16Hex(self, int16):
        lo_byte = int16 & 0x00FF
        hi_byte = (int16 & 0xFF00) >> 8
        all_bytes = (hi_byte << 8) | lo_byte
        return all_bytes

    def deg_to_32bitArray(self, deg):
        dec = int((deg + 1440) * (65536 + 65536) / (1440 + 1440) - 65536)
        HI_WORD = (dec & 0xFFFF0000) >> 16
        LO_WORD = dec & 0x0000FFFF
        return [HI_WORD, LO_WORD]

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr
        return q

    def get_wheels_travelled(self):
        registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
        l_pul_hi = registers[0]
        l_pul_lo = registers[1]
        r_pul_hi = registers[2]
        r_pul_lo = registers[3]
        l_pulse = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
        r_pulse = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))
        l_travelled = (
            (float(l_pulse) / self.cpr) *
            self.computation_left_wheel_radius * np.pi * 8
        )  # unit in meter
        r_travelled = (
            (float(r_pulse) / self.cpr)
            * self.computation_right_wheel_radius
            * np.pi
            * 8
        )  # unit in meter
        return l_travelled, r_travelled

    def modbus_fail_read_handler(self, ADDR, WORD):
        read_success = False
        reg = [None] * WORD
        while not read_success:
            result = self.client.read_holding_registers(
                ADDR, WORD, slave=self.ID)
            try:
                for i in range(WORD):
                    reg[i] = result.registers[i]
                read_success = True
            except AttributeError as e:
                self.get_logger().error(e)
                pass
        return reg

    def set_rpm_with_limit(self, left_rpm, right_rpm):
        if self.left_rpm_lim < left_rpm:
            left_rpm = self.left_rpm_lim
            self.get_logger().warn("RPM reach the limit.")
        elif left_rpm < -self.left_rpm_lim:
            left_rpm = -self.left_rpm_lim
            self.get_logger().warn("RPM reach the limit.")
        elif -self.deadband_rpm < left_rpm < self.deadband_rpm:
            left_rpm = 0

        if self.right_rpm_lim < right_rpm:
            right_rpm = self.right_rpm_lim
            self.get_logger().warn("RPM reach the limit.")
        elif right_rpm < -self.right_rpm_lim:
            right_rpm = -self.right_rpm_lim
            self.get_logger().warn("RPM reach the limit.")
        elif -self.deadband_rpm < right_rpm < self.deadband_rpm:
            right_rpm = 0

        left_bytes = self.int16Dec_to_int16Hex(int(left_rpm))
        right_bytes = self.int16Dec_to_int16Hex(int(right_rpm))
        result = self.client.write_registers(
            self.L_CMD_RPM, [left_bytes, right_bytes], slave=self.ID
        )

    def set_relative_angle(self, ang_L, ang_R):
        L_array = self.deg_to_32bitArray(ang_L / 4)
        R_array = self.deg_to_32bitArray(ang_R / 4)
        all_cmds_array = L_array + R_array
        result = self.client.write_registers(
            self.L_CMD_REL_POS_HI, all_cmds_array, slave=self.ID
        )

    def calculate_odometry(self):
        self.l_meter, self.r_meter = self.get_wheels_travelled()
        self.l_meter = self.l_meter - self.l_meter_init
        self.r_meter = (-1 * self.r_meter) - (-1 * self.r_meter_init)
        vl = (self.l_meter - self.prev_l_meter) / self.period
        vr = (self.r_meter - self.prev_r_meter) / self.period
        Wl = vl / self.computation_left_wheel_radius
        Wr = vr / self.computation_right_wheel_radius
        matrix = np.array(
            [
                [
                    self.computation_right_wheel_radius / 2,
                    self.computation_left_wheel_radius / 2,
                ],
                [
                    self.computation_right_wheel_radius / self.wheels_base_width,
                    -self.computation_left_wheel_radius / self.wheels_base_width,
                ],
            ]
        )
        vector = np.array([[Wr], [Wl]])
        input_vector = np.dot(matrix, vector)
        V = input_vector[0, 0]
        Wz = input_vector[1, 0]
        out_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        dirc_matrix = np.array(
            [
                [np.cos(self.state_vector[2, 0]) * self.period, 0],
                [np.sin(self.state_vector[2, 0]) * self.period, 0],
                [0, self.period],
            ]
        )
        out_vector = np.dot(out_matrix, self.state_vector) + np.dot(
            dirc_matrix, input_vector
        )
        x = out_vector[0, 0]
        y = out_vector[1, 0]
        theta = out_vector[2, 0]

        # -----Construct TF-----
        if self.publish_TF:
            self.t.header.stamp = self.get_clock().now().to_msg()
            self.t.header.frame_id = self.TF_header_frame
            self.t.child_frame_id = self.TF_child_frame
            self.t.transform.translation.x = x
            self.t.transform.translation.y = y
            self.t.transform.translation.z = 0.0
            rotation = self.quaternion_from_euler(0, 0, theta)
            self.t.transform.rotation.x = rotation[1]
            self.t.transform.rotation.y = rotation[2]
            self.t.transform.rotation.z = rotation[3]
            self.t.transform.rotation.w = rotation[0]
            self.br.sendTransform(self.t)

        # -----Construct Odom Message-----
        if self.publish_odom:
            self.odom_msg.header.stamp = self.get_clock().now().to_msg()
            self.odom_msg.header.frame_id = self.odom_header_frame
            self.odom_msg.child_frame_id = self.odom_child_frame
            self.odom_msg.pose.pose.position.x = x
            self.odom_msg.pose.pose.position.y = y
            self.odom_msg.pose.pose.position.z = 0.0
            orientation = self.quaternion_from_euler(0, 0, theta)
            self.odom_msg.pose.pose.orientation.x = orientation[1]
            self.odom_msg.pose.pose.orientation.y = orientation[2]
            self.odom_msg.pose.pose.orientation.z = orientation[3]
            self.odom_msg.pose.pose.orientation.w = orientation[0]
            self.odom_msg.pose.covariance[0] = 0.0001
            self.odom_msg.pose.covariance[7] = 0.0001
            self.odom_msg.pose.covariance[14] = 0.000001
            self.odom_msg.pose.covariance[21] = 0.000001
            self.odom_msg.pose.covariance[28] = 0.000001
            self.odom_msg.pose.covariance[35] = 0.0001
            self.odom_msg.twist.twist.linear.x = V
            self.odom_msg.twist.twist.linear.y = 0.0
            self.odom_msg.twist.twist.angular.z = Wz
            self.odom_pub.publish(self.odom_msg)

        self.state_vector[0, 0] = x
        self.state_vector[1, 0] = y
        self.state_vector[2, 0] = theta
        return x, y, theta

    def control_loop(self):
        start_time = time.perf_counter()
        if self.estop:
            # -----Emergency Stop-----
            if self.control_mode == 3:
                self.set_rpm_with_limit(0, 0)

            elif self.control_mode == 1:
                result = self.client.write_register(
                    self.CONTROL_REG, self.EMER_STOP, slave=self.ID
                )

            if not self.estop_reset_flag:
                self.get_logger().warn("####################")
                self.get_logger().warn("---EMERGENCY STOP---")
                self.get_logger().warn("####################")
            self.estop_reset_flag = True

        elif not self.estop:
            if self.estop_reset_flag:
                # -----Clear Alarm-----
                result = self.client.write_register(
                    self.CONTROL_REG, self.ALRM_CLR, slave=self.ID
                )

                # -----Disable Motor-----
                result = self.client.write_register(
                    self.CONTROL_REG, self.DOWN_TIME, slave=self.ID
                )

                # -----Enable Motor-----
                result = self.client.write_register(
                    self.CONTROL_REG, self.ENABLE, slave=self.ID
                )

                self.estop_reset_flag = False

            # -----Speed RPM Control-----
            if self.control_mode == 3:
                if self.got_twist_cmd:
                    self.left_rpm_cmd, self.right_rpm_cmd = self.twist_to_rpm(
                        self.linear_vel_cmd, self.angular_vel_cmd
                    )
                    self.set_rpm_with_limit(
                        self.left_rpm_cmd, self.right_rpm_cmd)
                    self.got_twist_cmd = False

                elif self.got_vel_cmd:
                    self.left_rpm_cmd, self.right_rpm_cmd = self.vel_to_rpm(
                        self.left_vel_cmd, self.right_vel_cmd
                    )
                    self.set_rpm_with_limit(
                        self.left_rpm_cmd, self.right_rpm_cmd)
                    self.got_vel_cmd = False

                elif self.got_vel_rpm_cmd:
                    self.set_rpm_with_limit(
                        self.left_rpm_cmd, self.right_rpm_cmd)
                    self.got_vel_rpm_cmd = False

                elif (
                    time.perf_counter() - self.last_subscribed_time
                ) > self.callback_timeout:
                    self.left_rpm_cmd = 0.0
                    self.right_rpm_cmd = 0.0
                    self.set_rpm_with_limit(
                        self.left_rpm_cmd, self.right_rpm_cmd)

            # -----Position Control-----
            elif self.control_mode == 1:
                if self.got_pos_deg_cmd:
                    self.set_relative_angle(
                        self.left_pos_deg_cmd, self.right_pos_deg_cmd
                    )

                    # -----Move Left Wheel-----
                    result = self.client.write_register(
                        self.CONTROL_REG, self.POS_L_START, slave=self.ID
                    )

                    # -----Move Right Wheel-----
                    result = self.client.write_register(
                        self.CONTROL_REG, self.POS_R_START, slave=self.ID
                    )

                    self.got_pos_deg_cmd = False

                elif self.got_pos_dist_cmd:
                    (
                        self.left_pos_deg_cmd,
                        self.right_pos_deg_cmd,
                    ) = self.dist_to_relative_angle(
                        self.left_pos_dist_cmd, self.right_pos_dist_cmd
                    )
                    self.set_relative_angle(
                        self.left_pos_deg_cmd, self.right_pos_deg_cmd
                    )

                    # -----Move Left Wheel-----
                    result = self.client.write_register(
                        self.CONTROL_REG, self.POS_L_START, slave=self.ID
                    )

                    # -----Move Right Wheel-----
                    result = self.client.write_register(
                        self.CONTROL_REG, self.POS_R_START, slave=self.ID
                    )

                    self.got_pos_dist_cmd = False

        # -----Odometry computation-----
        x, y, theta = self.calculate_odometry()
        self.period = time.perf_counter() - start_time
        self.prev_l_meter = self.l_meter
        self.prev_r_meter = self.r_meter

        # -----Debugging Feature-----
        if self.debug:
            self.get_logger().info(
                "x: %f | y: %f | yaw: %f" % (x, y, np.rad2deg(theta))
            )

        else:
            pass

        time.sleep(self.period)

    def exit_node(self):
        # -----Disable Motor-----
        result = self.client.write_register(
            self.CONTROL_REG, self.DOWN_TIME, slave=self.ID
        )


def main(args=None):
    try:
        rclpy.init(args=args)
        motor_driver_node = MotorDriverNode()
        while rclpy.ok():
            motor_driver_node.control_loop()
            rclpy.spin_once(motor_driver_node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        motor_driver_node.exit_node()
        motor_driver_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
