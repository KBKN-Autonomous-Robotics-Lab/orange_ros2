#include "waypoint_follower_client/waypoint_follower_client.h"


WaypointFollowerClient::WaypointFollowerClient() : Node("waypoint_follower_client")
{
  client_ptr_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
  is_valid_goal_handle = false;
}

void WaypointFollowerClient::sendGoals()
{
  auto goal_msg = FollowWaypoints::Goal();
  goal_msg.poses.resize(4);

  goal_msg.poses[0].header.frame_id = "map";
  goal_msg.poses[0].header.stamp = this->now();
  goal_msg.poses[0].pose.position.x = 2.0;
  goal_msg.poses[0].pose.position.y = 0.0;
  goal_msg.poses[0].pose.orientation.x = 0.0;
  goal_msg.poses[0].pose.orientation.y = 0.0;
  goal_msg.poses[0].pose.orientation.z = 1.0;
  goal_msg.poses[0].pose.orientation.w = 0.0;

  goal_msg.poses[1].header.frame_id = "map";
  goal_msg.poses[1].header.stamp = this->now();
  goal_msg.poses[1].pose.position.x = 0.0;
  goal_msg.poses[1].pose.position.y = 2.0;
  goal_msg.poses[1].pose.orientation.x = 0.0;
  goal_msg.poses[1].pose.orientation.y = 0.0;
  goal_msg.poses[1].pose.orientation.z = -0.7;
  goal_msg.poses[1].pose.orientation.w = 0.7;

  goal_msg.poses[2].header.frame_id = "map";
  goal_msg.poses[2].header.stamp = this->now();
  goal_msg.poses[2].pose.position.x = -2.0;
  goal_msg.poses[2].pose.position.y = 0.0;
  goal_msg.poses[2].pose.orientation.x = 0.0;
  goal_msg.poses[2].pose.orientation.y = 0.0;
  goal_msg.poses[2].pose.orientation.z = 0.0;
  goal_msg.poses[2].pose.orientation.w = 1.0;

  goal_msg.poses[3].header.frame_id = "map";
  goal_msg.poses[3].header.stamp = this->now();
  goal_msg.poses[3].pose.position.x = 0.0;
  goal_msg.poses[3].pose.position.y = -2.0;
  goal_msg.poses[3].pose.orientation.x = 0.0;
  goal_msg.poses[3].pose.orientation.y = 0.0;
  goal_msg.poses[3].pose.orientation.z = 0.7;
  goal_msg.poses[3].pose.orientation.w = 0.7;

  RCLCPP_INFO(this->get_logger(), "Sending goal");
  auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&WaypointFollowerClient::onGoalResponseReceived, this, _1);
  send_goal_options.feedback_callback = std::bind(&WaypointFollowerClient::onFeedbackReceived, this, _1, _2);
  send_goal_options.result_callback = std::bind(&WaypointFollowerClient::onResultReceived, this, _1);

  while (!is_valid_goal_handle)
  {
    auto result = client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(get_logger(), "Waiting for action server...");
    rclcpp::sleep_for(5s);
  }
}

void WaypointFollowerClient::onGoalResponseReceived(const GoalHandleFollowWaypoints::SharedPtr &future)
{
  auto goal_handle = future.get();
  is_valid_goal_handle = static_cast<bool>(goal_handle);
  if (!is_valid_goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void WaypointFollowerClient::onFeedbackReceived(GoalHandleFollowWaypoints::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "CurrentWaypoint = %d", feedback->current_waypoint);
  rclcpp::sleep_for(5s);
}

void WaypointFollowerClient::onResultReceived(const GoalHandleFollowWaypoints::WrappedResult &result) {
  result_code_ = result.code;

  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:RCLCPP_INFO(this->get_logger(), "Goal was succeeded");
    for (unsigned int i = 0; i < result.result->missed_waypoints.size(); i++)
    {
      RCLCPP_INFO(this->get_logger(), "missed waypoints: %d", result.result->missed_waypoints[i]);
    }
    break;
  case rclcpp_action::ResultCode::ABORTED:RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointFollowerClient>();
  node->sendGoals();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
