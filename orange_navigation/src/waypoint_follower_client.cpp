#include "waypoint_follower_client/waypoint_follower_client.h"

WaypointFollowerClient::WaypointFollowerClient()
    : Node("waypoint_follower_client") {
  client_ptr_ =
      rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
  is_valid_goal_handle_ = false;
  this->declare_parameter<std::string>("waypoints_file_path", "");
  this->get_parameter("waypoints_file_path", waypoint_file_path_);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&WaypointFollowerClient::checkGoalStatus, this));
}

void WaypointFollowerClient::sendGoals() {
  auto goal_msg = FollowWaypoints::Goal();

  YAML::Node waypoints_yaml = YAML::LoadFile(waypoint_file_path_);
  const auto& waypoints = waypoints_yaml["waypoints"];

  goal_msg.poses.resize(waypoints.size());

  for (size_t i = 0; i < waypoints.size(); i++)
  {
    goal_msg.poses[i].header.frame_id = "map";
    goal_msg.poses[i].header.stamp = this->now();
    goal_msg.poses[i].pose.position.x = waypoints[i]["pose"]["position"]["x"].as<double>();
    goal_msg.poses[i].pose.position.y = waypoints[i]["pose"]["position"]["y"].as<double>();
    goal_msg.poses[i].pose.orientation.x = waypoints[i]["pose"]["orientation"]["x"].as<double>();
    goal_msg.poses[i].pose.orientation.y = waypoints[i]["pose"]["orientation"]["y"].as<double>();
    goal_msg.poses[i].pose.orientation.z = waypoints[i]["pose"]["orientation"]["z"].as<double>();
    goal_msg.poses[i].pose.orientation.w = waypoints[i]["pose"]["orientation"]["w"].as<double>();
  }

  RCLCPP_INFO(this->get_logger(), "Sending goal");
  auto send_goal_options =
      rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&WaypointFollowerClient::onGoalResponseReceived, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&WaypointFollowerClient::onFeedbackReceived, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&WaypointFollowerClient::onResultReceived, this, _1);

  std::this_thread::sleep_for(1s);
  auto result = client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void WaypointFollowerClient::checkGoalStatus()
{
  auto now = std::chrono::steady_clock::now();
  if (now - last_goal_accept_time_ > std::chrono::seconds(1) && !is_valid_goal_handle_)
  {
    sendGoals();
  }
}

void WaypointFollowerClient::onGoalResponseReceived(
    const GoalHandleFollowWaypoints::SharedPtr &future) {
  auto goal_handle = future.get();
  is_valid_goal_handle_ = static_cast<bool>(goal_handle);
  if (!is_valid_goal_handle_) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void WaypointFollowerClient::onFeedbackReceived(
    GoalHandleFollowWaypoints::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
  RCLCPP_INFO(this->get_logger(), "CurrentWaypoint = %d",
              feedback->current_waypoint);
  rclcpp::sleep_for(1s);
}

void WaypointFollowerClient::onResultReceived(
    const GoalHandleFollowWaypoints::WrappedResult &result) {
  result_code_ = result.code;

  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Goal was succeeded");
    for (unsigned int i = 0; i < result.result->missed_waypoints.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "missed waypoints: %d",
                  result.result->missed_waypoints[i]);
    }
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointFollowerClient>();
  node->sendGoals();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
