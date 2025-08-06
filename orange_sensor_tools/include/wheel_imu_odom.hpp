#ifndef WHEEL_IMU_ODOM_HPP
#define WHEEL_IMU_ODOM_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdomFusionNode : public rclcpp::Node
{
public:
  OdomFusionNode();

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  static void getYawFromQuaternion(
    const geometry_msgs::msg::Quaternion & q, double & roll, double & pitch, double & yaw);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string imu_topic_;
  std::string odom_topic_;
  std::string fused_odom_topic_;

  std::string odom_header_frame_;
  std::string odom_child_frame_;
  std::string TF_header_frame_;
  std::string TF_child_frame_;

  double scale_factor_;
  double pitch_diff_th_;
  bool publish_odom_;
  bool publish_TF_;
  bool debug_;

  rclcpp::Time prev_time_;
  bool imu_received_;
  bool baseline_initialized_;

  double roll_;
  double pitch_;
  double yaw_;
  double baseline_pitch_;

  double position_x_;
  double position_y_;
  double position_z_;

  double orientation_x_;
  double orientation_y_;
  double orientation_z_;
  double orientation_w_;

  double ang_vel_x_;
  double ang_vel_y_;
  double ang_vel_z_;
};

#endif  // WHEEL_IMU_ODOM_HPP
