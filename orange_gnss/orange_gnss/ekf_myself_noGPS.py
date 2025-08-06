#!/usr/bin/env python3
import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time


class ExtendedKalmanFilter(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        self.GTheta = None
        self.GTheta0 = None
        self.odom2theta = 0
        self.G1Thetayaw0 = 0
        self.DTheta = 0
        self.w = None
        self.Q = None  # Process noise covariance
        self.H = None
        self.R = None
        self.R1 = 6e-2  # 0.06 odom1
        self.R2 = 4e-2  # 0.04 odom2
        self.R3 = 4     # odom1 Theta
        self.R4 = 6     # odom2 theta
        self.P = None  # Initial covariance
        self.XX = None
        self.prev_time = None
        self.prev_pos = None
        self.Speed = 0
        self.SmpTime = 0.1
        self.odom2XY = None
        self.conut = 0
        self.GOffset = 0
        self.offsetyaw = 0
        self.combineyaw = 0
        self.robot_yaw = 0
        self.combyaw = 0
        self.robot_orientationz = 0
        self.robot_orientationw = 0

        self.sub_a = self.create_subscription(
            Odometry, '/odom', self.sensor_a_callback, 10)
        self.sub_b = self.create_subscription(
            Odometry, '/odom_fast', self.sensor_b_callback, 10)

        self.declare_parameter("publish_TF", False)
        self.ekf_publish_TF = self.get_parameter(
            "publish_TF").get_parameter_value().bool_value

        self.t = TransformStamped()
        self.br = tf2_ros.TransformBroadcaster(self)

        self.fused_pub = self.create_publisher(Odometry, '/fusion/odom', 10)
        self.fused_msg = Odometry()

        self.timer = self.create_timer(0.1, self.publish_fused_value)

        self.get_logger().info("Start ekf_myself_noGPS node")
        self.get_logger().info("-------------------------")

    def orientation_to_yaw(self, z, w):
        yaw = np.arctan2(2.0 * (w * z), 1.0 - 2.0 * (z ** 2))
        return yaw

    def yaw_to_orientation(self, yaw):
        orientation_z = np.sin(yaw / 2.0)
        orientation_w = np.cos(yaw / 2.0)
        return orientation_z, orientation_w

    def sensor_a_callback(self, data):

        # current_time = self.get_clock().now().to_msg()
        diff_time_stamp = Clock(clock_type=ClockType.ROS_TIME).now()
        current_time = diff_time_stamp.nanoseconds / 1000000000

        if self.prev_time is not None:
            self.SmpTime = current_time - self.prev_time
        else:
            self.SmpTime = 0.1
        self.prev_time = current_time

        current_pos = np.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y
        ])
        if self.prev_pos is not None:
            distance = np.linalg.norm(current_pos - self.prev_pos)
            self.Speed = distance / self.SmpTime
        else:
            self.Speed = 0
        self.prev_pos = current_pos

        self.GTheta = self.orientation_to_yaw(
            data.pose.pose.orientation.z, data.pose.pose.orientation.w)

    def sensor_b_callback(self, data):
        self.odom2XY = np.array(
            [data.pose.pose.position.x, data.pose.pose.position.y])

        self.odom2theta = self.orientation_to_yaw(
            data.pose.pose.orientation.z, data.pose.pose.orientation.w)

        self.DTheta = self.odom2theta - self.G1Thetayaw0
        self.G1Thetayaw0 = self.odom2theta

    def initialize(self, GTheta, SmpTime):
        self.GTheta0 = GTheta
        self.XX = np.array([0, 0, np.cos(GTheta), np.sin(GTheta)])
        self.w = np.array([(1.379e-3)**2, (0.03 * np.pi / 180 * SmpTime)**2])
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.Q = np.array(
            [[(1.379e-3)**2, 0], [0, (0.03 * np.pi / 180 * SmpTime)**2]])
        G0 = np.array([[1, 0], [0, 0], [0, 0], [0, 1]])
        self.P = G0 @ self.Q @ G0.T

    def initialize_odom2(self, XY, GTheta, SmpTime):
        self.GTheta0 = GTheta
        self.XX = np.array(
            [XY[0], XY[1], np.cos(GTheta), np.sin(GTheta)])
        self.w = np.array([(1.379e-3)**2, (0.03 * np.pi / 180 * SmpTime)**2])
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.Q = np.array(
            [[(1.379e-3)**2, 0], [0, (0.03 * np.pi / 180 * SmpTime)**2]])
        G0 = np.array([[1, 0], [0, 0], [0, 0], [0, 1]])
        self.P = G0 @ self.Q @ G0.T

    def KalfXY(self, Speed, SmpTime, GTheta, R1, R2):
        if self.H is None:
            self.initialize(GTheta, SmpTime)

        self.R = np.array([[R1, 0], [0, R2]])

        DTheta = GTheta - self.GTheta0
        self.GTheta0 = GTheta
        # equation of state F G
        F = np.array([
            [1, 0, Speed * SmpTime *
                np.cos(DTheta), -Speed * SmpTime * np.sin(DTheta)],
            [0, 1, Speed * SmpTime *
                np.sin(DTheta), Speed * SmpTime * np.cos(DTheta)],
            [0, 0, np.cos(DTheta), -np.sin(DTheta)],
            [0, 0, np.sin(DTheta), np.cos(DTheta)]
        ])

        G = np.array([
            [np.cos(GTheta), -Speed * SmpTime * np.sin(GTheta)],
            [np.sin(GTheta), Speed * SmpTime * np.cos(GTheta)],
            [0, -np.sin(GTheta)],
            [0, np.cos(GTheta)]
        ])

        self.XX = F @ self.XX + G @ self.w

        return self.XX[:2]

    def Kalfodom2XY(self, Speed, SmpTime, GTheta, XY, R1, R2):
        if self.H is None:
            self.initialize_odom2(XY, GTheta, SmpTime)

        self.R = np.array([[R1, 0], [0, R2]])

        DTheta = GTheta - self.GTheta0
        self.GTheta0 = GTheta

        # equation of state F G
        F = np.array([
            [1, 0, Speed * SmpTime *
                np.cos(DTheta), -Speed * SmpTime * np.sin(DTheta)],
            [0, 1, Speed * SmpTime *
                np.sin(DTheta), Speed * SmpTime * np.cos(DTheta)],
            [0, 0, np.cos(DTheta), -np.sin(DTheta)],
            [0, 0, np.sin(DTheta), np.cos(DTheta)]
        ])

        G = np.array([
            [np.cos(GTheta), -Speed * SmpTime * np.sin(GTheta)],
            [np.sin(GTheta), Speed * SmpTime * np.cos(GTheta)],
            [0, -np.sin(GTheta)],
            [0, np.cos(GTheta)]
        ])

        Y = np.array([XY[0], XY[1]])

        self.XX = F @ self.XX  # filter equation
        self.P = F @ self.P @ F.T + G @ self.Q @ G.T  # Prior Error Covariance
        # kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(self.H @
                                              self.P @ self.H.T + self.R)
        self.XX = self.XX + K @ (Y - self.H @ self.XX)  # estimated value
        self.P = self.P - K @ self.H @ self.P  # Posterior Error Covariance

        return self.XX[:2]

    def combine_yaw(self, Dtheta, theta1, theta2, w1, w2):
        if abs(Dtheta) < 5 * math.pi / 180:
            if theta1 < 0:
                theta1 += 2 * math.pi
            if theta2 < 0:
                theta2 += 2 * math.pi

            x1, y1 = w1 * math.cos(theta1), w1 * math.sin(theta1)
            x2, y2 = w2 * math.cos(theta2), w2 * math.sin(theta2)
            x_sum = (x1 + x2) / (w1 + w2)
            y_sum = (y1 + y2) / (w1 + w2)
            theta_sum = math.atan2(y_sum, x_sum)

            if theta_sum > math.pi:
                theta_sum -= 2 * math.pi
            elif theta_sum < -math.pi:
                theta_sum += 2 * math.pi
        else:
            theta_sum = theta1

        return theta_sum

    def calculate_offset(self, combyaw, GTheta, GPStheta):
        abs_GTheta = abs(GTheta)
        abs_GPStheta = abs(GPStheta)
        abs_combyaw = abs(combyaw)
        pi = math.pi

        deference = abs_GTheta + abs_GPStheta

        if GTheta > 0 and GPStheta < 0 and combyaw > 0:
            self.GOffset = -(GTheta - combyaw)
        elif GTheta > 0 and GPStheta < 0 and combyaw < 0:
            self.GOffset = -(GTheta + abs_combyaw)
        elif GTheta > 0 and GPStheta > 0 and combyaw > 0 and GTheta > GPStheta:
            self.GOffset = -(abs_combyaw - abs_GTheta)
        elif GTheta < 0 and GPStheta < 0 and combyaw < 0 and GTheta > GPStheta:
            self.GOffset = -(GTheta + abs_combyaw)
        elif GTheta < 0 and GPStheta > 0 and combyaw > 0:
            self.GOffset = abs_GTheta + combyaw
        elif GTheta < 0 and GPStheta > 0 and combyaw < 0:
            self.GOffset = abs_GTheta - abs_combyaw
        elif GTheta > 0 and GPStheta > 0 and combyaw > 0 and GTheta < GPStheta:
            self.GOffset = combyaw - GTheta
        elif GTheta < 0 and GPStheta < 0 and combyaw < 0 and GTheta < GPStheta:
            self.GOffset = abs_GTheta - abs_combyaw
        elif GTheta > 0 and GPStheta < 0 and combyaw > 0 and deference > pi:
            self.GOffset = combyaw - GTheta
        elif GTheta > 0 and GPStheta < 0 and combyaw < 0 and deference > pi:
            self.GOffset = pi - GTheta + pi - abs_combyaw
        elif GTheta < 0 and GPStheta > 0 and combyaw > 0 and deference > pi:
            self.GOffset = -((pi - combyaw) + (pi - abs_GTheta))
        elif GTheta < 0 and GPStheta > 0 and combyaw < 0 and deference > pi:
            self.GOffset = -(abs_combyaw - abs_GTheta)

        if abs(self.GOffset) > 5 * pi / 180:  # not -0.0872 ~ 0.0872
            self.GOffset = 0
            self.get_logger().warn("GOffset warning")

        # self.get_logger().info(f"GOffset: {self.GOffset}") ok

        return self.GOffset

    def publish_fused_value(self):
        if self.Speed is not None and self.SmpTime is not None and self.GTheta is not None:
            if self.odom2XY is not None:
                fused_value = self.Kalfodom2XY(
                    self.Speed, self.SmpTime, self.GTheta, self.odom2XY, self.R1, self.R2)
                self.conut += 1
                if self.conut % 20 == 0:
                    self.combyaw = self.combine_yaw(
                        self.DTheta, self.GTheta, self.odom2theta, self.R3, self.R4)
                    self.offsetyaw = self.calculate_offset(
                        self.combyaw, self.GTheta, self.odom2theta)

                self.robot_yaw = self.GTheta + self.offsetyaw
                if self.robot_yaw < -np.pi:
                    self.robot_yaw += 2 * np.pi
                elif self.robot_yaw > np.pi:
                    self.robot_yaw -= 2 * np.pi

                robot_orientation = self.yaw_to_orientation(self.robot_yaw)
                self.robot_orientationz = robot_orientation[0]
                self.robot_orientationw = robot_orientation[1]

                self.fused_msg.pose.pose.position.x = float(fused_value[0])
                self.fused_msg.pose.pose.position.y = float(fused_value[1])
                self.fused_msg.pose.pose.orientation.z = float(
                    self.robot_orientationz)
                self.fused_msg.pose.pose.orientation.w = float(
                    self.robot_orientationw)
            else:
                fused_value = self.KalfXY(
                    self.Speed, self.SmpTime, self.GTheta, self.R1, self.R2)
                self.robot_yaw = self.GTheta + self.offsetyaw
                if self.robot_yaw < -np.pi:
                    self.robot_yaw += 2 * np.pi
                elif self.robot_yaw > np.pi:
                    self.robot_yaw -= 2 * np.pi
                robot_orientation = self.yaw_to_orientation(self.robot_yaw)
                self.robot_orientationz = robot_orientation[0]
                self.robot_orientationw = robot_orientation[1]

                self.fused_msg.pose.pose.position.x = float(fused_value[0])
                self.fused_msg.pose.pose.position.y = float(fused_value[1])
                self.fused_msg.pose.pose.orientation.z = float(
                    self.robot_orientationz)
                self.fused_msg.pose.pose.orientation.w = float(
                    self.robot_orientationw)

            self.fused_msg.header.stamp = self.get_clock().now().to_msg()
            self.fused_msg.header.frame_id = "odom"
            # self.get_logger().info(f"ekf position: {fused_value}")
            # self.get_logger().info(f"orientation:{self.robot_orientationz},{self.robot_orientationw}")
            self.fused_pub.publish(self.fused_msg)

            self.Speed = None
            self.SmpTime = None
            self.GTheta = None

            if self.ekf_publish_TF:
                self.t.header.stamp = self.get_clock().now().to_msg()
                self.t.header.frame_id = "odom"
                self.t.child_frame_id = "base_footprint"
                self.t.transform.translation.x = float(fused_value[0])
                self.t.transform.translation.y = float(fused_value[1])
                self.t.transform.translation.z = 0.0
                self.t.transform.rotation.x = 0.0
                self.t.transform.rotation.y = 0.0
                self.t.transform.rotation.z = float(self.robot_orientationz)
                self.t.transform.rotation.w = float(self.robot_orientationw)
                self.br.sendTransform(self.t)


def main(args=None):
    rclpy.init(args=args)
    ekf = ExtendedKalmanFilter()
    rclpy.spin(ekf)
    ekf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
