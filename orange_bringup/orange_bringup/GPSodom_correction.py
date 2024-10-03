#!/usr/bin/env python3
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class GPSodom_correction(Node):
    def __init__(self):
        super().__init__('GPSodom_correction')

        self.create_subscription(
            Odometry, "/odom/gps", self.odomgps_callback, 1)
        self.create_subscription(
            Imu, "/livox/imu", self.imu_callback, 1)

        self.odom_pub = self.create_publisher(
            Odometry, "/odom/gps/corr", 10)
        self.odom_msg = Odometry()

        self.x = None
        self.prev_x = 0
        self.y = None
        self.prev_y = 0
        self.satelites = 0

        self.velocity = np.array([0.0, 0.0, 0.0])
        self.prev_time = None
        self.is_static = False

        self.imu_processing_rate = 1
        self.last_imu_process_time = None

        self.timer = self.create_timer(1.0 / 3.0, self.publish_correct_GPSodom)

    def odomgps_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.satelites = msg.pose.covariance[0]

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = current_time
            return

        if self.last_imu_process_time is not None and (current_time - self.last_imu_process_time).nanoseconds * 1e-9 < self.imu_processing_rate:
            return

        dt = (current_time - self.prev_time).nanoseconds * 1e-9

        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        self.velocity[0] += acc_x * dt
        self.velocity[1] += acc_y * dt
        self.velocity[2] += (acc_z + 9.81) * dt

        speed = math.sqrt(
            self.velocity[0]**2 + self.velocity[1]**2 + self.velocity[2]**2)

        # self.get_logger().info(f"speed: {speed}")

        if abs(speed) <= 0.41:  # 0.1389
            self.is_static = True
        else:
            self.is_static = False

        self.velocity = np.array([0.0, 0.0, 0.0])
        self.prev_time = current_time
        self.last_imu_process_time = current_time

    def publish_correct_GPSodom(self):
        if self.x is not None and self.y is not None and self.satelites is not None:
            if self.is_static:  # stationary state
                self.odom_msg.header.stamp = self.get_clock().now().to_msg()
                self.odom_msg.header.frame_id = "odom"
                self.odom_msg.child_frame_id = "base_footprint"
                self.odom_msg.pose.pose.position.x = float(self.prev_x)
                self.odom_msg.pose.pose.position.y = float(self.prev_y)
                self.odom_msg.pose.pose.position.z = 0.0
                self.odom_msg.pose.pose.orientation.x = 0.0
                self.odom_msg.pose.pose.orientation.y = 0.0
                self.odom_msg.pose.pose.orientation.z = 0.0
                # Number of satellites
                self.odom_msg.pose.covariance[0] = self.satelites
                self.get_logger().info("stationary state")
            else:
                self.odom_msg.header.stamp = self.get_clock().now().to_msg()
                self.odom_msg.header.frame_id = "odom"
                self.odom_msg.child_frame_id = "base_footprint"
                self.odom_msg.pose.pose.position.x = float(self.x)
                self.odom_msg.pose.pose.position.y = float(self.y)
                self.odom_msg.pose.pose.position.z = 0.0
                self.odom_msg.pose.pose.orientation.x = 0.0
                self.odom_msg.pose.pose.orientation.y = 0.0
                self.odom_msg.pose.pose.orientation.z = 0.0
                # Number of satellites
                self.odom_msg.pose.covariance[0] = self.satelites
                self.prev_x = self.x
                self.prev_y = self.y

            self.odom_pub.publish(self.odom_msg)
            self.x = None
            self.y = None
            self.satelites = None


def main(args=None):
    rclpy.init(args=args)
    correct = GPSodom_correction()
    rclpy.spin(correct)
    correct.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
