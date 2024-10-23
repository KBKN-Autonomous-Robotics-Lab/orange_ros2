#!/usr/bin/env python3
import math

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu


class MovingBaseNode(Node):
    def __init__(self):
        super().__init__('movingbase')
        self.HEADER = 6
        self.count = 0
        self.first_heading = 0

        self.declare_parameter('port', '/dev/sensors/GNSSrover')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('time_out', 1.0)

        self.port = self.get_parameter(
            'port').get_parameter_value().string_value
        self.baudrate = self.get_parameter(
            'baud').get_parameter_value().integer_value
        self.time_out = self.get_parameter(
            'time_out').get_parameter_value().double_value

        # self.sub_movingbase = self.create_subscription(Imu,'movingbase_yaw',self.movingbase_callback,10)

        self.heading_pub = self.create_publisher(Imu, 'movingbase/quat', 10)
        self.movingbase_msg = Imu()
        self.movingbase_data = None

        self.get_logger().info("Start movingbase_yaw_to_quat node")
        self.get_logger().info("---------------------------------")

    # def movingbase_callback(self, data):
    #    self.movingbase_data = data

    def readrelposned(self):
        ackPacket = [b'\xB5', b'\x62', b'\x01', b'\x3C', b'\x00', b'\x00']
        i = 0
        payloadlength = 6
        with serial.Serial(self.port, self.baudrate, timeout=self.time_out) as ser:
            while i < payloadlength + 8:
                incoming_char = ser.read()
                if (i < 3) and (incoming_char == ackPacket[i]):
                    i += 1
                elif i == 3:
                    ackPacket[i] = incoming_char
                    i += 1
                elif i == 4:
                    ackPacket[i] = incoming_char
                    i += 1
                elif i == 5:
                    ackPacket[i] = incoming_char
                    payloadlength = int.from_bytes(
                        ackPacket[4] + ackPacket[5], byteorder='little', signed=False)
                    i += 1
                elif i > 5:
                    ackPacket.append(incoming_char)
                    i += 1

        if self.checksum(ackPacket, payloadlength):
            # self.get_logger().info("Checksum OK")
            nowpoint_info = self.parse_heading(ackPacket)
            return nowpoint_info

    def checksum(self, ackPacket, payloadlength):
        CK_A = 0
        CK_B = 0
        for i in range(2, payloadlength + 6):
            CK_A += int.from_bytes(ackPacket[i],
                                   byteorder='little', signed=False)
            CK_B += CK_A
        CK_A &= 0xff
        CK_B &= 0xff
        if (CK_A == int.from_bytes(ackPacket[-2], byteorder='little', signed=False)) and \
           (CK_B == int.from_bytes(ackPacket[-1], byteorder='little', signed=False)):
            return True
        else:
            self.get_logger().error("ACK Checksum Failure")
            self.get_logger().error("!!! -movingbase receive error- !!!")
            return False

    def parse_heading(self, ackPacket):
        nowPoint = []

        # GPStime
        byteoffset = 4 + self.HEADER
        bytevalue = ackPacket[byteoffset]
        for i in range(1, 4):
            bytevalue += ackPacket[byteoffset + i]
        time = int.from_bytes(bytevalue, byteorder='little', signed=True)
        gpstime = time / 1000
        nowPoint.append(gpstime)  # 0

        # Carrier solution status
        flags = int.from_bytes(
            ackPacket[60 + self.HEADER], byteorder='little', signed=True)
        gnssFixOK = flags & (1 << 0)  # gnssFixOK
        # carrSoln: 0=no carrier, 1=float, 2=fix
        carrSoln = (flags & (0b11 << 3)) >> 3
        nowPoint.append(gnssFixOK)  # 1
        nowPoint.append(carrSoln)  # 2

        # relPosHeading
        byteoffset = 24 + self.HEADER
        bytevalue = ackPacket[byteoffset]
        for i in range(1, 4):
            bytevalue += ackPacket[byteoffset + i]
        heading = int.from_bytes(bytevalue, byteorder='little', signed=True)
        nowPoint.append(heading / 100000)  # 3

        return nowPoint

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

    def movingbase_publish_msg(self):

        nowpoint = self.readrelposned()
        if nowpoint is not None:

            heading = nowpoint[3] + 90
            if heading >= 360:
                heading -= 360

            self.get_logger().info(f"robotheading: {heading}")

            if self.count == 0:
                self.first_heading = heading
                self.count = 1

            relative_heading = heading - self.first_heading
            if relative_heading < 0:
                relative_heading += 360

            if relative_heading > 180:
                relative_heading -= 360

            movingbaseyaw = relative_heading * (math.pi / 180)

            roll, pitch = 0.0, 0.0
            yaw = movingbaseyaw

            q = self.quaternion_from_euler(roll, pitch, yaw)
            # self.get_logger().info(f"Quaternion: {q}")

            self.movingbase_msg.header.stamp = self.get_clock().now().to_msg()
            self.movingbase_msg.header.frame_id = "imu_link"
            self.movingbase_msg.orientation.x = q[1]
            self.movingbase_msg.orientation.y = q[2]
            self.movingbase_msg.orientation.z = -q[3]  # -z
            self.movingbase_msg.orientation.w = q[0]
            self.movingbase_msg.orientation_covariance[0] = heading

            self.heading_pub.publish(self.movingbase_msg)
            self.movingbase_data = None
        else:
            self.get_logger().error("!!! -movingbase data error- !!!")


def main(args=None):
    rclpy.init(args=args)
    movingbase_node = MovingBaseNode()
    rate = movingbase_node.create_rate(3)

    while rclpy.ok():
        movingbase_node.movingbase_publish_msg()
        rclpy.spin_once(movingbase_node)
        rate.sleep()

    movingbase_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
