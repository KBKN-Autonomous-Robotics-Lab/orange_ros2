#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class CLASMovingBaseCombiner(Node):
    def __init__(self):
        super().__init__('clas_moving_base_combiner')

        self.sub1 = Subscriber(self, Odometry, "/odom/gps")
        self.sub2 = Subscriber(self, Imu, "movingbase/quat")

        self.mf = ApproximateTimeSynchronizer([self.sub1, self.sub2], queue_size=10, slop=0.5)
        self.mf.registerCallback(self.callback)
        
        self.odom_pub = self.create_publisher(Odometry, "/CLAS_movingbase", 10)
        self.odom_msg = Odometry()

        self.CLAS_position = None        
        self.movingbase_yaw = None   
        
    def callback(self, msg1, msg2):
        self.CLAS_position = msg1
        self.movingbase_yaw = msg2
    
    def publish_combined_odom(self):
        if self.CLAS_position is not None and self.movingbase_yaw is not None:
            self.odom_msg.header.stamp = self.get_clock().now().to_msg()
            self.odom_msg.header.frame_id = "odom"
            self.odom_msg.child_frame_id = "base_footprint"
            self.odom_msg.pose.pose.position.x = self.CLAS_position.pose.pose.position.x
            self.odom_msg.pose.pose.position.y = self.CLAS_position.pose.pose.position.y
            self.odom_msg.pose.pose.position.z = self.CLAS_position.pose.pose.orientation.w  # Number of satellites
            self.odom_msg.pose.pose.orientation.x = 0
            self.odom_msg.pose.pose.orientation.y = 0
            self.odom_msg.pose.pose.orientation.z = self.movingbase_yaw.orientation.z
            self.odom_msg.pose.pose.orientation.w = self.movingbase_yaw.orientation.w
            self.odom_msg.pose.covariance = [0.0001, 0, 0, 0, 0, 0, 
                                             0, 0.0001, 0, 0, 0, 0, 
                                             0, 0, 0.0001, 0, 0, 0, 
                                             0, 0, 0, 0.0001, 0, 0, 
                                             0, 0, 0, 0, 0.0001, 0, 
                                             0, 0, 0, 0, 0, 0.0001]
            self.odom_pub.publish(self.odom_msg)
        else:
            self.get_logger().warn("Data missing: CLAS_position or movingbase_yaw is None")
            
def main(args=None):
    rclpy.init(args=args)
    combiner = CLASMovingBaseCombiner()
    rclpy.spin(combiner)
    combiner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
