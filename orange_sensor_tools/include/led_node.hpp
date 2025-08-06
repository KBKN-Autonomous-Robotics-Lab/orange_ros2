#ifndef LED_NODE_HPP
#define LED_NODE_HPP

#include <serial/serial.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class LedController : public rclcpp::Node
{
public:
  LedController();
  ~LedController();

private:
  void navCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void sendSerialCommand();

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  //rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  serial::Serial serial_port_;

  std::string port_name_;
  int baud_rate_;
  std::string nav_topic_;
  std::string estop_topic_;
  bool nav_state_;
  bool estop_state_;
  std::string led_state_;
};

#endif  // LED_NODE_HPP
