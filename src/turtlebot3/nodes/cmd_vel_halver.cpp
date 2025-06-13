#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVelHalver : public rclcpp::Node
{
public:
    CmdVelHalver() : Node("cmd_vel_halver")
    {
        // 구독자: /cmd_vel
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_sim", 10,
            std::bind(&CmdVelHalver::cmdVelCallback, this, std::placeholders::_1)
        );

        // 퍼블리셔: /cmd_vel_half
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_half", 10);

        RCLCPP_INFO(this->get_logger(), "cmd_vel_halver 노드 시작됨");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto new_msg = geometry_msgs::msg::Twist();

        // 선형 속도 절반
        new_msg.linear.x = msg->linear.x * 0.9;
        new_msg.linear.y = msg->linear.y * 0.9;
        new_msg.linear.z = msg->linear.z * 0.9;

        // 각속도 절반
        new_msg.angular.x = msg->angular.x * 0.5;
        new_msg.angular.y = msg->angular.y * 0.5;
        new_msg.angular.z = msg->angular.z * 0.5;

        publisher_->publish(new_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelHalver>());
    rclcpp::shutdown();
    return 0;
}
