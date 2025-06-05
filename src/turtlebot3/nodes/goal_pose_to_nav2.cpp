#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class OdomToNavGoal : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  OdomToNavGoal()
  : Node("odom_to_nav_goal"), goal_sent_(false)
  {
    // 액션 클라이언트 생성
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // /odom 토픽 구독
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&OdomToNavGoal::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "OdomToNavGoal node started. Waiting for odom...");
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  bool goal_sent_;  // 한 번만 보낼 때 사용

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (goal_sent_) return;  // 한 번만 전송

    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available.");
      return;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";  // tf 구성에 따라 "odom"으로 바꿔도 됨
    goal.header.stamp = this->get_clock()->now();
    goal.pose = msg->pose.pose;

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal;

    RCLCPP_INFO(this->get_logger(), "Sending goal from odom: (%.2f, %.2f)",
                goal.pose.position.x, goal.pose.position.y);

    action_client_->async_send_goal(goal_msg);
    goal_sent_ = true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomToNavGoal>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
