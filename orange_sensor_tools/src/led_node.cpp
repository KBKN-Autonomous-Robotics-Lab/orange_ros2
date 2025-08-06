#include "led_node.hpp"

using namespace std::chrono_literals;

LedController::LedController() : Node("LED_node"), nav_state_(false), estop_state_(false)
{
  port_name_ = this->declare_parameter<std::string>("serial_port", "/dev/sensors/LED");
  baud_rate_ = this->declare_parameter<int>("baud_rate", 115200);
  nav_topic_ = this->declare_parameter<std::string>("nav_topic", "/nav_state");
  estop_topic_ = this->declare_parameter<std::string>("estop_topic", "/estop");

  try {
    serial_port_.setPort(port_name_);
    serial_port_.setBaudrate(baud_rate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    serial_port_.setTimeout(to);
    serial_port_.open();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
  }

  nav_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    nav_topic_, 10, std::bind(&LedController::navCallback, this, std::placeholders::_1));
  estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    estop_topic_, 10, std::bind(&LedController::estopCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(500ms, std::bind(&LedController::sendSerialCommand, this));

  if (serial_port_.isOpen()) {
    serial_port_.write("ON\n");
  }
}

LedController::~LedController()
{
  if (serial_port_.isOpen()) {
    serial_port_.write("OFF\n");
    serial_port_.close();
  }
}

void LedController::navCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  nav_state_ = msg->data;
}

void LedController::estopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  estop_state_ = msg->data;
}

void LedController::sendSerialCommand()
{
  led_state_ = (nav_state_ && !estop_state_) ? "FLASH" : "SOLID";

  if (serial_port_.isOpen()) {
    try {
      serial_port_.write(led_state_ + "\n");
      // RCLCPP_INFO(this->get_logger(), "Sent: %s", led_state_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", e.what());
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Serial port is NOT open!");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedController>());
  rclcpp::shutdown();
  return 0;
}
