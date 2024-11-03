#!/usr/bin/env python3
import math

import rclpy
import serial
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix


class lonlat_To_Odom(Node):
    def __init__(self):
        super().__init__('gps_data_acquisition')

        self.declare_parameter('Position_magnification', 1.675)
        # self.declare_parameter('heading', 180)

        self.Position_magnification = self.get_parameter(
            'Position_magnification').get_parameter_value().double_value
        # self.theta = self.get_parameter('heading').get_parameter_value().double_value

        self.movingase_sub = self.create_subscription(
            Imu, "movingbase/quat", self.movingbase_callback, 1)
        self.fix_sub = self.create_subscription(
            NavSatFix, "fix", self.fix_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, "/odom/gps", 10)
        self.odom_msg = Odometry()

        self.initial_coordinate = None
        self.fix_data = None

        self.count = 0

        self.latitude = None
        self.longitude = None
        self.satelites = None
        self.theta = None

        self.timer = self.create_timer(1.0 / 3.0, self.publish_lonlat_to_odom)

        self.get_logger().info("Start lonlat_to_odom node")
        self.get_logger().info("-------------------------")

    def fix_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.satelites = data.position_covariance[0]

    def movingbase_callback(self, msg):
        if self.count == 0:
            self.theta = msg.orientation_covariance[0]
            self.count = 1

    def conversion(self, coordinate, origin, theta):
        ido = coordinate[0]
        keido = coordinate[1]
        ido0 = origin[0]
        keido0 = origin[1]

        # self.get_logger().info(f"theta: {theta}")

        a = 6378137
        f = 35/10439
        e1 = 734/8971
        e2 = 127/1547
        n = 35/20843
        a0 = 1
        a2 = 102/40495
        a4 = 1/378280
        a6 = 1/289634371
        a8 = 1/204422462123
        pi180 = 71/4068
        # %math.pi/180
        d_ido = ido - ido0
        d_keido = keido - keido0
        rd_ido = d_ido * pi180
        rd_keido = d_keido * pi180
        r_ido = ido * pi180
        r_keido = keido * pi180
        r_ido0 = ido0 * pi180
        W = math.sqrt(1-(e1**2)*(math.sin(r_ido)**2))
        N = a / W
        t = math.tan(r_ido)
        ai = e2*math.cos(r_ido)

       # %===Y===%
        S = a*(a0*r_ido - a2*math.sin(2*r_ido)+a4*math.sin(4*r_ido) -
               a6*math.sin(6*r_ido)+a8*math.sin(8*r_ido))/(1+n)
        S0 = a*(a0*r_ido0-a2*math.sin(2*r_ido0)+a4*math.sin(4*r_ido0) -
                a6*math.sin(6*r_ido0)+a8*math.sin(8*r_ido0))/(1+n)
        m0 = S/S0
        B = S-S0
        y1 = (rd_keido**2)*N*math.sin(r_ido)*math.cos(r_ido)/2
        y2 = (rd_keido**4)*N*math.sin(r_ido) * \
            (math.cos(r_ido)**3)*(5-(t**2)+9*(ai**2)+4*(ai**4))/24
        y3 = (rd_keido**6)*N*math.sin(r_ido)*(math.cos(r_ido)**5) * \
            (61-58*(t**2)+(t**4)+270*(ai**2)-330*(ai**2)*(t**2))/720
        gps_y = self.Position_magnification * m0 * (B + y1 + y2 + y3)

       # %===X===%
        x1 = rd_keido*N*math.cos(r_ido)
        x2 = (rd_keido**3)*N*(math.cos(r_ido)**3)*(1-(t**2)+(ai**2))/6
        x3 = (rd_keido**5)*N*(math.cos(r_ido)**5) * \
            (5-18*(t**2)+(t**4)+14*(ai**2)-58*(ai**2)*(t**2))/120
        gps_x = self.Position_magnification * m0 * (x1 + x2 + x3)

        # point = (gps_x, gps_y)Not match

        degree_to_radian = math.pi / 180
        r_theta = theta * degree_to_radian
        h_x = math.cos(r_theta) * gps_x - math.sin(r_theta) * gps_y
        h_y = math.sin(r_theta) * gps_x + math.cos(r_theta) * gps_y
        point = (h_y, -h_x)
        # point = (-h_y, h_x)
        # point = (h_y, -h_x)

        return point

    def publish_lonlat_to_odom(self):
        lonlat = [self.latitude, self.longitude]
        if lonlat[0] is not None and lonlat[1] is not None and self.theta is not None:
            if lonlat[0] != 0 and lonlat[1] != 0:
                if self.initial_coordinate is None:
                    self.initial_coordinate = [lonlat[0], lonlat[1]]
                GPSxy = self.conversion(
                    lonlat, self.initial_coordinate, self.theta)
                # self.get_logger().info(f"GPSxy: {GPSxy}")
                # self.get_logger().info(f"lonlat[4]: {lonlat[4]}")

                self.odom_msg.header.stamp = self.get_clock().now().to_msg()
                self.odom_msg.header.frame_id = "odom"
                self.odom_msg.child_frame_id = "base_footprint"
                self.odom_msg.pose.pose.position.x = GPSxy[0]
                self.odom_msg.pose.pose.position.y = GPSxy[1]
                self.odom_msg.pose.pose.position.z = 0.0
                self.odom_msg.pose.pose.orientation.x = 0.0
                self.odom_msg.pose.pose.orientation.y = 0.0
                self.odom_msg.pose.pose.orientation.z = 0.0
                # Number of satellites
                self.odom_msg.pose.covariance[0] = self.satelites

                self.odom_pub.publish(self.odom_msg)


def main(args=None):
    rclpy.init(args=args)
    lonlattodom = lonlat_To_Odom()
    rclpy.spin(lonlattodom)
    lonlattodom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
