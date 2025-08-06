#!/usr/bin/env python3
import math
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

class GPS_heading_Data(Node):
    def __init__(self):
        super().__init__('gps_data_acquisition')

        self.declare_parameter('port', '/dev/sensors/GNSS_UM982')
        self.declare_parameter('baud', 115200)

        self.dev_name = self.get_parameter(
            'port').get_parameter_value().string_value
        self.serial_baud = self.get_parameter(
            'baud').get_parameter_value().integer_value

        self.heading_pub = self.create_publisher(Imu, 'movingbase/quat', 10)
        self.movingbase_msg = Imu()

        self.timer = self.create_timer(1.0, self.movingbase_publish_msg)
        self.count = 0
        
        self.get_logger().info("Start get_movingbase_quat_ttyUSB node")
        self.get_logger().info("-------------------------")

    def get_gps_heading(self, dev_name):
        try:
            serial_port = serial.Serial(dev_name, self.serial_baud)
        except serial.SerialException as serialerror:
            self.get_logger().error(f"Serial error: {serialerror}")
            return None

        initial_letters_outdoor = b"$GNHDT"
        initial_letters_indoor = b"$GPHDT"

        while(1):
            line = serial_port.readline()
            #self.get_logger().info(f"line: {line}")
            talker_ID_indoor = line.find(initial_letters_indoor)
            talker_ID_outdoor = line.find(initial_letters_outdoor)            
            if talker_ID_indoor != -1:
                #self.get_logger().info("GPHDT ok")
                #line = line[(talker_ID_indoor-1):]
                gps_data = line.split(b",")
                #self.get_logger().info(f"gps_data: {gps_data}")
                heading = float(gps_data[1])
                if heading is None:
                    self.get_logger().error("not GPS heading data")
                    heading = 0
                break
            if talker_ID_outdoor != -1:
                #self.get_logger().info("GNHDT ok")
                #line = line[(talker_ID_outdoor-1):]
                gps_data = line.split(b",")
                #self.get_logger().info(f"gps_data: {gps_data}")
                heading = float(gps_data[1])
                if heading is None:
                    self.get_logger().error("not GPS heading data")
                    heading = 0
                break          
                
        serial_port.close()

        return heading

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
        real_heading = self.get_gps_heading(self.dev_name)
        #self.get_logger().info(f"real_heading: {real_heading}")
        if real_heading is not None and real_heading != 0:

            robotheading = real_heading + 90
            if robotheading >= 360:
                robotheading -= 360

            #self.get_logger().info(f"robotheading: {robotheading}")

            if self.count == 0:
                self.get_logger().info(f"!!!----------robotheading: {robotheading} deg----------!!!")
                self.first_heading = robotheading
                self.count = 1

            relative_heading = robotheading - self.first_heading
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
            self.movingbase_msg.orientation_covariance[0] = robotheading
            self.heading_pub.publish(self.movingbase_msg)
        else:
            self.movingbase_msg.header.stamp = self.get_clock().now().to_msg()
            self.movingbase_msg.header.frame_id = "imu_link"
            self.movingbase_msg.orientation.x = 0.0
            self.movingbase_msg.orientation.y = 0.0
            self.movingbase_msg.orientation.z = 0.0
            self.movingbase_msg.orientation.w = 0.0
            self.heading_pub.publish(self.movingbase_msg)
            self.get_logger().error("!!!!-not movingbase data-!!!!")

def main(args=None):
    rclpy.init(args=args)
    GPS_heading_Data_node = GPS_heading_Data()
    rclpy.spin(GPS_heading_Data_node)
    GPS_heading_Data_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
