#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header


class GPSData(Node):
    def __init__(self):
        super().__init__('gps_data_acquisition')

        self.declare_parameter('port', '/dev/sensors/GNSS_UM982')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('country_id', 0)

        self.dev_name = self.get_parameter(
            'port').get_parameter_value().string_value
        self.serial_baud = self.get_parameter(
            'baud').get_parameter_value().integer_value
        self.country_id = self.get_parameter(
            'country_id').get_parameter_value().integer_value

        self.lonlat_pub = self.create_publisher(NavSatFix, "/fix", 1)
        self.lonlat_msg = NavSatFix()

        self.initial_coordinate = None
        self.fix_data = None

        self.timer = self.create_timer(1.0, self.publish_GPS_lonlat)

        self.get_logger().info("Start get_lonlat node")
        self.get_logger().info("-------------------------")

    def get_gps(self, dev_name, country_id):
        # interface with sensor device(as a serial port)
        try:
            serial_port = serial.Serial(dev_name, self.serial_baud)
        except serial.SerialException as serialerror:
            self.get_logger().error(f"Serial error: {serialerror}")
            return None
        
        # country info 
        if country_id == 0:   # Japan
            initial_letters = b"GNGGA"
        elif country_id == 1: # USA
            initial_letters = b"GPGGA"
        else:                 # not certain
            initial_letters = None

#    gps_data = ["$G?GGA", 
#                "UTC time", 
#                "Latitude (ddmm.mmmmm)", 
#                "latitude type (south/north)", 
#                "Longitude (ddmm.mmmmm)", 
#                "longitude type (east longitude/west longitude)", 
#                "Fixtype", 
#                "Number of satellites used for positioning", 
#                "HDOP", 
#                "Altitude", 
#                "M(meter)", 
#                "Elevation", 
#                "M(meter)", 
#                "", 
#                "checksum"]
    
        line = serial_port.readline()
        talker_ID = line.find(initial_letters)
        if talker_ID != -1:
            line = line[(talker_ID-1):]
            gps_data = line.split(b",")
            Fixtype_data = int(gps_data[6])
            if Fixtype_data != 0:
                satelitecount_data = int(gps_data[7])###
                if Fixtype_data != 0:
                    latitude_data = float(gps_data[2]) / 100.0  # ddmm.mmmmm to dd.ddddd
                    if gps_data[3] == b"S":#south
                        latitude_data *= -1
                    longitude_data = float(gps_data[4]) / 100.0  # ddmm.mmmmm to dd.ddddd
                    if gps_data[5] == b"W":#west
                        longitude_data *= -1
                    altitude_data = float(gps_data[9])
                else :
                    #not fix data
                    latitude_data = 0
                    longitude_data = 0
                    altitude_data = 0
                    satelitecount_data = 0
                    self.get_logger().error("!--not fix data--!")
            else :
            #no GPS data
                latitude_data = 0
                longitude_data = 0
                altitude_data = 0
                satelitecount_data = 0
                self.get_logger().error("!--not GPS data--!")
        else:
            self.get_logger().error("!--not GPS data--!")
            return None
        serial_port.close()

        gnggadata = (Fixtype_data,latitude_data,longitude_data,altitude_data,satelitecount_data)
        return gnggadata


    def publish_GPS_lonlat(self):
        lonlat = self.get_gps(self.dev_name, self.country_id)
        if lonlat:
            self.lonlat_msg.header = Header()
            self.lonlat_msg.header.frame_id = "gps"
            self.lonlat_msg.header.stamp = self.get_clock().now().to_msg()

            self.lonlat_msg.status.status = NavSatStatus.STATUS_FIX if lonlat[
                0] != 0 else NavSatStatus.STATUS_NO_FIX
            self.lonlat_msg.latitude = float(lonlat[1])
            self.lonlat_msg.longitude = float(lonlat[2])
            self.lonlat_msg.altitude = float(lonlat[3])

            self.lonlat_pub.publish(self.lonlat_msg)
            # self.get_logger().info(f"Published GPS data: {lonlat}")
        else:
            self.get_logger().error("!!!!-gps data error-!!!!")


def main(args=None):
    rclpy.init(args=args)
    gpslonlat = GPSData()
    rclpy.spin(gpslonlat)
    gpslonlat.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
