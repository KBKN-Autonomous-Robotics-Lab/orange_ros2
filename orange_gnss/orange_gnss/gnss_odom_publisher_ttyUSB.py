#!/usr/bin/env python3
import rclpy
import serial
import math
import tkinter as tk
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
import threading
import time
from my_msgs.srv import Avglatlon

class GPSData(Node):
    def __init__(self):
        super().__init__('gps_data_acquisition')

        self.declare_parameter('port', '/dev/sensors/GNSS_UM982')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('country_id', 0)
        self.declare_parameter('Position_magnification', 1.675)
        self.declare_parameter('heading', 180.0)

        self.dev_name = self.get_parameter('port').get_parameter_value().string_value
        self.serial_baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.country_id = self.get_parameter('country_id').get_parameter_value().integer_value
        self.Position_magnification = self.get_parameter('Position_magnification').get_parameter_value().double_value
        #self.theta = self.get_parameter('heading').get_parameter_value().double_value

        self.initial_coordinate = None
        self.fix_data = None
        self.count = 0
        
        self.odom_pub = self.create_publisher(Odometry, "/odom/UM982", 10)
        self.odom_msg = Odometry()
        
        self.lonlat_pub = self.create_publisher(NavSatFix, "/fix", 1)
        self.lonlat_msg = NavSatFix()
        
        self.initialized = False  # 平均初期座標が取得できたかどうか
        self.timer = self.create_timer(1.0, self.publish_GPS_lonlat_quat)
        
        # service client
        self.client = self.create_client(Avglatlon, 'send_avg_gps')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available...")

        self.get_logger().info("Start get_lonlat quat node")
        self.get_logger().info("-------------------------")

        # tkinter GUI setup
        self.root = tk.Tk()
        self.root.title("GPS Data Acquisition")
        self.start_button = tk.Button(self.root, text="Start GPS Acquisition", command=self.start_gps_acquisition)
        self.start_button.pack()

        self.gps_acquisition_thread = None
        self.is_acquiring = False

    # service client
    def send_request(self):
        request = Avglatlon.Request()
        request.avg_lat = self.initial_coordinate[0]  # ← average lat
        request.avg_lon = self.initial_coordinate[1]  # ← average lon
        request.theta = self.theta

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
        
    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('サービス送信成功')
            else:
                self.get_logger().warn('サービスは受け取られましたが、処理は失敗しました')
        except Exception as e:
            self.get_logger().error(f'サービス呼び出し失敗: {e}')
    
    # gps data collect
    def start_gps_acquisition(self):
        if not self.is_acquiring:
            self.is_acquiring = True
            self.gps_acquisition_thread = threading.Thread(target=self.acquire_gps_data)
            self.gps_acquisition_thread.start()

    def acquire_gps_data(self):
        lat_sum = 0.0
        lon_sum = 0.0
        heading_sum = 0.0
        count = 0
        #serial_port = serial.Serial(self.dev_name, self.serial_baud)
        #line = serial_port.readline()

        start_time = time.time()
        while time.time() - start_time < 10:  # 10 seconds
            GPS_data = self.get_gps_quat(self.dev_name, self.country_id)
            #gps_data = line.split(b",")
            if GPS_data and GPS_data[1] != 0 and GPS_data[2] != 0:
                lat_sum += GPS_data[1]
                lon_sum += GPS_data[2]
                heading_sum += float(GPS_data[5])
                count += 1
            time.sleep(0.1)  # Slight delay to avoid overwhelming the GPS device

        if count > 0:
            self.initial_coordinate = [lat_sum / count, lon_sum / count]
            self.theta = (heading_sum / count) - 90
            self.initialized = True
            self.get_logger().info(f"Initial coordinate set to: {self.initial_coordinate}")
            self.get_logger().info(f"Initial theta set to: {self.theta}")
            self.send_request()
        self.is_acquiring = False

    def get_gps_quat(self, dev_name, country_id):
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
                    Fixtype_data = 0
                    latitude_data = 0
                    longitude_data = 0
                    altitude_data = 0
                    satelitecount_data = 0
                    self.get_logger().error("!--not fix data--!")
            else :
            #no GPS data
                Fixtype_data = 0
                latitude_data = 0
                longitude_data = 0
                altitude_data = 0
                satelitecount_data = 0
                self.get_logger().error("!--not GPS data--!")          
        
        serial_port.close()

        gnggadata = (Fixtype_data,latitude_data,longitude_data,altitude_data,satelitecount_data,heading)
        return gnggadata

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
    
    
    def heading_to_quat(self ,real_heading):

        robotheading = real_heading - 90
        if robotheading >= 360:
            robotheading -= 360

        self.get_logger().info(f"real_heading: {real_heading}")
        self.get_logger().info(f"robotheading: {robotheading}")

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
            
        return q
    
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
        #point = (-h_y, h_x)
        point = (h_y, -h_x)

        return point
    
    def publish_GPS_lonlat_quat(self):
        if not self.initialized:
            # 初期化が完了していないので何もしない
            return        
        
        GPS_data = self.get_gps_quat(self.dev_name, self.country_id)
        #gnggadata = (Fixtype_data,latitude_data,longitude_data,altitude_data,satelitecount_data,heading)
        if GPS_data and GPS_data[1] != 0 and GPS_data[2] != 0:
            self.satelite = GPS_data[4]
            lonlat = [GPS_data[1], GPS_data[2]]
            
            # publish fix topic
            #self.lonlat_msg.header = Header()
            #self.lonlat_msg.frame_id = "gps"
            #self.lonlat_msg.header.stamp = self.get_clock().now().to_msg()
            
            #self.lonlat_msg.status.status = NavSatStatus.STATUS_FIX if lonlat[
            #    0] != 0 else NavSatStatus.STATUS_NO_FIX
            #self.lonlat_msg.latitude = GPS_data[1] # ido
            #self.lonlat_msg.longitude = GPS_data[2] # keido
            #self.lonlat_msg.altitude = GPS_data[3] # koudo
            
            #self.lonlat_pub.publish(self.lonlat_msg)
            # self.get_logger().info(f"Published GPS data: {lonlat}")           
            
            if self.initial_coordinate is None:
                self.initial_coordinate = [GPS_data[1], GPS_data[2]]        
            GPSxy = self.conversion(lonlat, self.initial_coordinate, self.theta)
            GPSquat = self.heading_to_quat(GPS_data[5])       

            self.odom_msg.header.stamp = self.get_clock().now().to_msg()
            self.odom_msg.header.frame_id = "odom"
            self.odom_msg.child_frame_id = "base_footprint"
            self.odom_msg.pose.pose.position.x = GPSxy[0]
            self.odom_msg.pose.pose.position.y = GPSxy[1]

            self.odom_msg.pose.pose.orientation.x = GPSquat[1]
            self.odom_msg.pose.pose.orientation.y = GPSquat[2]
            self.odom_msg.pose.pose.orientation.z = -GPSquat[3]
            self.odom_msg.pose.pose.orientation.w = GPSquat[0]
            # Number of satellites
            self.odom_msg.pose.covariance[0] = float(self.satelite)
            self.odom_pub.publish(self.odom_msg)
        else:
            self.get_logger().error("!!!!-NOT RECIEVE-gps data error-!!!!")


def main(args=None):
    #rclpy.init(args=args)
    #gpslonlat = GPSData()
    #rclpy.spin(gpslonlat)
    #gpslonlat.root.mainloop()
    #gpslonlat.destroy_node()
    #rclpy.shutdown()
    rclpy.init(args=args)
    gpslonlat = GPSData()
    
    ros_thread = threading.Thread(target=rclpy.spin, args=(gpslonlat,))
    ros_thread.start()

    gpslonlat.root.mainloop()  # tkinter GUI表示

    gpslonlat.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
