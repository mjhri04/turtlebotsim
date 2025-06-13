// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/int32.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <cmath>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>

// class GoalMover : public rclcpp::Node
// {
// public:
//   GoalMover() : Node("goal_mover")
//   {
//     vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_sim", 10);
//     trigger_sub_ = this->create_subscription<std_msgs::msg::Int32>(
//       "/trigger_yolo", 10,
//       std::bind(&GoalMover::trigger_callback, this, std::placeholders::_1));
//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//       "/sim_odom", 10,
//       std::bind(&GoalMover::odom_callback, this, std::placeholders::_1));

//     timer_ = this->create_wall_timer(
//       std::chrono::milliseconds(100),
//       std::bind(&GoalMover::move_to_goal, this));

//     goals_ = {
//       std::make_pair(-2, -1.3),
//       std::make_pair(-2, 0.7),
//       std::make_pair(-0.5,  0.7),
//       std::make_pair(-0.5,  -1.3)
//     };

//     goal_active_ = false;
//   }

// private:
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
//   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   std::vector<std::pair<double, double>> goals_;
//   double robot_x_, robot_y_;
//   bool goal_active_;
//   std::pair<double, double> current_goal_;

//   void trigger_callback(const std_msgs::msg::Int32::SharedPtr msg)
//   {
//     int idx = msg->data - 1;
//     if (idx >= 0 && idx < static_cast<int>(goals_.size())) {
//       current_goal_ = goals_[idx];
//       goal_active_ = true;
//       RCLCPP_INFO(this->get_logger(), "Goal %d activated (%.2f, %.2f)",
//                   idx + 1, current_goal_.first, current_goal_.second);
//     }
//   }

//   double robot_yaw_;

//   void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
//   {
//     robot_x_ = msg->pose.pose.position.x;
//     robot_y_ = msg->pose.pose.position.y;

//     auto orientation = msg->pose.pose.orientation;
//     tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
//     double roll, pitch;
//     tf2::Matrix3x3(q).getRPY(roll, pitch, robot_yaw_);
//   }

//   void move_to_goal()
//   {
//     if (!goal_active_) return;

//     double dx = current_goal_.first - robot_x_;
//     double dy = current_goal_.second - robot_y_;
//     double dist = std::sqrt(dx*dx + dy*dy);

//     double target_yaw = std::atan2(dy, dx);
//     double yaw_error = target_yaw - robot_yaw_;

//     while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
//     while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

//     geometry_msgs::msg::Twist cmd;

//     const double dist_thresh = 0.2;
//     const double yaw_thresh = 0.3;

//     if (dist < dist_thresh && std::abs(yaw_error) < yaw_thresh) {
//       cmd.linear.x = 0.0;
//       cmd.angular.z = 0.0;
//       goal_active_ = false;
//       RCLCPP_INFO(this->get_logger(), "Goal reached.");
//     } else {
//       if (std::abs(yaw_error) > yaw_thresh) {
//         cmd.linear.x = 0.0;
//         cmd.angular.z = 0.5 * yaw_error;
//       } else {
//         cmd.linear.x = 0.2;
//         cmd.angular.z = 0.2 * yaw_error;  // 직진하면서 보정
//       }
//       RCLCPP_INFO(this->get_logger(), "yaw_error: %.3f, dist: %.3f", yaw_error, dist);
//     }

//     vel_pub_->publish(cmd);
//   }
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<GoalMover>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }




// // goal_mover_node.cpp
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/int32.hpp>
// #include <nav2_msgs/action/navigate_to_pose.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>

// using std::placeholders::_1;
// using NavigateToPose = nav2_msgs::action::NavigateToPose;

// class GoalMover : public rclcpp::Node {
// public:
//   GoalMover() : Node("oal_pose_to_nav2") {
//     trigger_sub_ = this->create_subscription<std_msgs::msg::Int32>(
//       "/trigger_yolo", 10, std::bind(&GoalMover::trigger_callback, this, _1));

//     action_client_ = rclcpp_action::create_client<NavigateToPose>(
//       this, "navigate_to_pose");

//     goals_ = {
//       std::make_pair(-2.0, -1.3),
//       std::make_pair(-2.0,  0.7),
//       std::make_pair(-0.5,  0.7),
//       std::make_pair(-0.5, -1.3)
//     };

//     RCLCPP_INFO(this->get_logger(), "GoalMover node started");
//   }

// private:
//   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_sub_;
//   rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
//   std::vector<std::pair<double, double>> goals_;

//   void trigger_callback(const std_msgs::msg::Int32::SharedPtr msg) {
//     int index = msg->data - 1;
//     if (index < 0 || index >= static_cast<int>(goals_.size())) {
//       RCLCPP_WARN(this->get_logger(), "Invalid goal index: %d", msg->data);
//       return;
//     }

//     if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
//       RCLCPP_ERROR(this->get_logger(), "navigate_to_pose server not available");
//       return;
//     }

//     auto [x, y] = goals_[index];

//     NavigateToPose::Goal goal_msg;
//     goal_msg.pose.header.frame_id = "map";
//     goal_msg.pose.header.stamp = this->now();
//     goal_msg.pose.pose.position.x = x;
//     goal_msg.pose.pose.position.y = y;
//     goal_msg.pose.pose.orientation.w = 1.0;

//     auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//     action_client_->async_send_goal(goal_msg, send_goal_options);

//     RCLCPP_INFO(this->get_logger(), "Sent goal #%d → (%.2f, %.2f)", msg->data, x, y);
//   }
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<GoalMover>());
//   rclcpp::shutdown();
//   return 0;
// }















#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class GoalMover : public rclcpp::Node
{
public:
  GoalMover() : Node("goal_mover")
  {
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_sim", 10);
    trigger_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/trigger_yolo", 10,
      std::bind(&GoalMover::trigger_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GoalMover::process, this));

    goals_ = {
      std::make_pair(-2.0, -1.3),
      std::make_pair(-2.0, 0.7),
      std::make_pair(-0.5, 0.7),
      std::make_pair(-0.5, -1.3)
    };

    mode_ = IDLE;
  }

private:
  enum Mode { IDLE, ROTATING, MOVING, DONE };
  Mode mode_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::pair<double, double>> goals_;
  double target_angle_;
  int tick_;

  std::vector<double> rotation_speed_ = {0.05, -0.32, -0.25, -0.25};  // rad/s
  std::vector<double> move_speed_ = {-0.3, -0.38, -0.3, -0.38};      // m/s

  std::vector<int> ROTATE_TICKS = {90, 70, 70, 70};  // 1초 (100ms * 20)
  std::vector<int> MOVE_TICKS = {82, 40, 60, 40};    // 3초 (100ms * 30)

  int current_goal_idx_ = -1;  // 0부터 시작


  void trigger_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int idx = msg->data - 1;
    if (idx >= 0 && idx < static_cast<int>(goals_.size())) {
      current_goal_idx_ = idx;
      tick_ = 0;
      mode_ = ROTATING;

      auto [x, y] = goals_[idx];
      target_angle_ = std::atan2(y, x);
      RCLCPP_INFO(this->get_logger(), "Goal #%d: (%.2f, %.2f), angle %.2f rad", idx + 1, x, y, target_angle_);
    }
  }

  void process()
  {
    if (current_goal_idx_ == -1) return;

    geometry_msgs::msg::Twist cmd;

    switch (mode_) {
      case IDLE:
        return;

      case ROTATING:
        if (tick_ < ROTATE_TICKS[current_goal_idx_]) {
          cmd.angular.z = rotation_speed_[current_goal_idx_];
          tick_++;
        } else {
          tick_ = 0;
          mode_ = MOVING;
          RCLCPP_INFO(this->get_logger(), "Rotation complete. Start moving.");
        }
        break;

      case MOVING:
        if (tick_ < MOVE_TICKS[current_goal_idx_]) {
          cmd.linear.x = move_speed_[current_goal_idx_];
          tick_++;
        } else {
          mode_ = DONE;
          RCLCPP_INFO(this->get_logger(), "Movement complete.");
        }
        break;

      case DONE:
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        break;
    }

    vel_pub_->publish(cmd);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalMover>());
  rclcpp::shutdown();
  return 0;
}
