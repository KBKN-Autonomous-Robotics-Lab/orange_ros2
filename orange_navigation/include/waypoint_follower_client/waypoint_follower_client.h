#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

class WaypointFollowerClient : public rclcpp::Node
{
  private:
    rclcpp_action::Client<FollowWaypoints>::SharedPtr client_ptr_;
    rclcpp_action::ResultCode result_code_;
    bool is_valid_goal_handle;

  public:
    WaypointFollowerClient();
    void sendGoals();
    void onGoalResponseReceived(const GoalHandleFollowWaypoints::SharedPtr &future);
    void onFeedbackReceived(GoalHandleFollowWaypoints::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
    void onResultReceived(const GoalHandleFollowWaypoints::WrappedResult &result);
};
