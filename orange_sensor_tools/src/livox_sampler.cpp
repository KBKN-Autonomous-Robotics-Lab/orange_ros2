#include "rclcpp/rclcpp.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
//#include "sensor_msgs/msg/point_cloud2.hpp"

class LivoxSampler : public rclcpp::Node
{
public:
  LivoxSampler() : Node("livox_sampler_node")
  {
    this->declare_parameter("sampling_factor", 5);
    this->get_parameter("sampling_factor", sampling_factor_);
    
    sampled_pcd_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
        "/sampled/lidar",10);
    livox_pcd_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "/livox/lidar", 10, std::bind(&LivoxSampler::callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "***** Start livox_sampler_node *****");
  }

private:
  void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    if (counter_ % sampling_factor_ == 0) {
      livox_ros_driver2::msg::CustomMsg sampled_msg = *msg;
      sampled_msg.point_num = msg->point_num / sampling_factor_;
      sampled_msg.points.resize(sampled_msg.point_num);
      sampled_pcd_pub_->publish(sampled_msg);
    }
    counter_++;
    if (counter_ >= 10000) counter_ = 0;
  }
  
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr sampled_pcd_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_pcd_sub_;
  int sampling_factor_;
  int counter_ = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivoxSampler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

