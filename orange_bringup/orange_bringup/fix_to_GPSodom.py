#!/usr/bin/env python3
import math

import rclpy
import serial
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GPSDataToOdom(Node):
    def __init__(self):
        super().__init__('gps_data_acquisition')

        self.declare_parameter('port', '/dev/sensors/GNSSbase')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('country_id', 0)
        self.declare_parameter('Position_magnification', 1.675)
        self.declare_parameter('heading', 180)

        self.dev_name = self.get_parameter(
            'port').get_parameter_value().string_value
        self.serial_baud = self.get_parameter(
            'baud').get_parameter_value().integer_value
        self.country_id = self.get_parameter(
            'country_id').get_parameter_value().integer_value
        self.Position_magnification = self.get_parameter(
            'Position_magnification').get_parameter_value().double_value
        self.theta = self.get_parameter(
            'heading').get_parameter_value().double_value

#        self.create_subscription(Imu, "movingbase/quat", self.movingbase_callback, 1)
#        self.fix_sub = self.create_subscription(NavSatFix, "fix", self.fix_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom/gps", 10)
        self.odom_msg = Odometry()

        self.initial_coordinate = None
        self.fix_data = None
#        self.count = 0

        self.timer = self.create_timer(1.0 / 3.0, self.publish_GPSodom)

#    def fix_callback(self, data):
#        self.fix_data = data

#    def movingbase_callback(self, msg):
#         if self.count == 0:
#             self.theta = msg.orientation_covariance[0]
#             self.count = 1

    def get_gps(self, dev_name, country_id):
        try:
            serial_port = serial.Serial(dev_name, self.serial_baud)
        except serial.SerialException as serialerror:
            self.get_logger().error(f"Serial error: {serialerror}")
            return None

        initial_letters = None
        if country_id == 0:   # Japan
            initial_letters = "$GNGGA"
        elif country_id == 1:  # USA
            initial_letters = "$GPGGA"

        while True:
            line = serial_port.readline().decode('latin-1')
            gps_data = line.split(',')
            if gps_data[0] == initial_letters:
                break

        # self.get_logger().info(f"GPS Data: {gps_data}")
        Fixtype_data = int(gps_data[6])
        # self.get_logger().info(f"Fix Type: {Fixtype_data}")
        if Fixtype_data != 0:
            satelitecount_data = float(gps_data[7])
            # self.get_logger().info(f"Satellite Count: {satelitecount_data}")
            # ddmm.mmmmm to dd.ddddd
            latitude_data = float(gps_data[2]) / 100.0
            if gps_data[3] == 'S':  # south
                latitude_data *= -1
            # ddmm.mmmmm to dd.ddddd
            longitude_data = float(gps_data[4]) / 100.0
            if gps_data[5] == 'W':  # west
                longitude_data *= -1
            altitude_data = float(gps_data[9])
        else:
            latitude_data = 0
            longitude_data = 0
            altitude_data = 0
            satelitecount_data = 0

        serial_port.close()

        gnggadata = (Fixtype_data, latitude_data, longitude_data,
                     altitude_data, satelitecount_data)
        # self.get_logger().info(f"Current Latitude and Longitude (Fixtype, latitude, longitude, altitude): {gnggadata}")

        return gnggadata

    def conversion(self, coordinate, origin, theta):
        ido = coordinate[1]
        keido = coordinate[2]
        ido0 = origin[0]
        keido0 = origin[1]

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
        point = (-h_y, h_x)
        # point = (h_y, -h_x)

        return point

    def publish_GPSodom(self):
        lonlat = self.get_gps(self.dev_name, self.country_id)
        if lonlat is not None:
            if lonlat[1] != 0 and lonlat[2] != 0:
                if self.initial_coordinate is None:
                    self.initial_coordinate = [lonlat[1], lonlat[2]]
                GPSxy = self.conversion(
                    lonlat, self.initial_coordinate, self.theta)
                # self.get_logger().info(f"GPSxy: {GPSxy}")
                # self.get_logger().info(f"lonlat[4]: {lonlat[4]}")
                satellites = lonlat[4]

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
                self.odom_msg.pose.covariance[0] = satellites

                self.odom_pub.publish(self.odom_msg)


def main(args=None):
    rclpy.init(args=args)
    gtodom = GPSDataToOdom()
    rclpy.spin(gtodom)
    gtodom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
