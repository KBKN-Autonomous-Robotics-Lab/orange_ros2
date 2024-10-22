#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LidarSampler : public rclcpp::Node
{
public:
  LidarSampler() : Node("lidar_sampler_node")
  {
    this->declare_parameter("sampling_factor", 10);
    this->get_parameter("sampling_factor", sampling_factor_);
    
    post_pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sampled/lidar",10);
    prior_pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/converted_pointcloud2", 10, std::bind(&LidarSampler::callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "***** Start lidar_sampler_node *****");
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (counter_ % sampling_factor_ == 0) {
      post_pcd_pub_->publish(*msg);
    }
    counter_++;
    if (counter_ >= 10000) counter_ = 0;
  }
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr post_pcd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr prior_pcd_sub_;
  int sampling_factor_;
  int counter_ = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarSampler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

