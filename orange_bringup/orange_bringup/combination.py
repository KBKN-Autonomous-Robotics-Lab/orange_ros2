#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class CLASMovingBaseCombiner(Node):
    def __init__(self):
        super().__init__('clas_moving_base_combiner')

        self.create_subscription(
            Odometry, "/odom/gps", self.odomgps_callback, 1)
        self.create_subscription(
            Imu, "movingbase/quat", self.movingbase_callback, 1)

        self.odom_pub = self.create_publisher(
            Odometry, "/odom_CLAS_movingbase", 10)
        self.odom_msg = Odometry()

        self.x = 0
        self.y = 0
        self.satelite = 0

        self.orientationz = 0
        self.orientationw = 0

        self.timer = self.create_timer(1.0, self.publish_combined_odom)

        self.get_logger().info("Start combination node")
        self.get_logger().info("----------------------")

    def odomgps_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.satelite = msg.pose.covariance[0]

    def movingbase_callback(self, msg):
        self.orientationz = msg.orientation.z
        self.orientationw = msg.orientation.w

    def publish_combined_odom(self):
        if self.x is not None and self.y is not None and self.satelite is not None:
            if self.x != 0 and self.y != 0 and self.orientationz != 0 and self.orientationw != 0:
                self.odom_msg.header.stamp = self.get_clock().now().to_msg()
                self.odom_msg.header.frame_id = "odom"
                self.odom_msg.child_frame_id = "base_footprint"
                self.odom_msg.pose.pose.position.x = float(self.x)
                self.odom_msg.pose.pose.position.y = float(self.y)

                self.odom_msg.pose.pose.orientation.x = 0.0
                self.odom_msg.pose.pose.orientation.y = 0.0
                self.odom_msg.pose.pose.orientation.z = float(
                    self.orientationz)
                self.odom_msg.pose.pose.orientation.w = float(
                    self.orientationw)
                # Number of satellites
                self.odom_msg.pose.covariance[0] = float(self.satelite)
                self.odom_pub.publish(self.odom_msg)
                self.x = None
                self.y = None
                self.satelite = None


def main(args=None):
    rclpy.init(args=args)
    combiner = CLASMovingBaseCombiner()
    rclpy.spin(combiner)
    combiner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
