#pragma once

#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollowWaypoints =
    rclcpp_action::ClientGoalHandle<FollowWaypoints>;

class WaypointFollowerClient : public rclcpp::Node {
private:
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_ptr_;
  rclcpp_action::ResultCode result_code_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool is_valid_goal_handle_;
  std::string waypoint_file_path_;
  std::chrono::steady_clock::time_point last_goal_accept_time_;

public:
  WaypointFollowerClient();
  void sendGoals();
  void checkGoalStatus();
  void
  onGoalResponseReceived(const GoalHandleFollowWaypoints::SharedPtr &future);
  void onFeedbackReceived(
      GoalHandleFollowWaypoints::SharedPtr,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
  void onResultReceived(const GoalHandleFollowWaypoints::WrappedResult &result);
};
