/// This is meant to convert the twist message to a steer message. This will need to be tuned.
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int8.hpp"

class TwistToSteer : public rclcpp::Node {
public:
    TwistToSteer() : Node("twist_to_steer") {
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "robot_twist", 10, std::bind(&TwistToSteer::twist_callback, this, std::placeholders::_1)
        );
        steer_publisher_ = this->create_publisher<std_msgs::msg::Int8>("control/steer", 10);
    }
private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto steer_msg = std_msgs::msg::Int8();
        // Convert the twist message to a steer message
        // This is a simple conversion, you may need to adjust the scaling factor
        steer_msg.data = static_cast<int8_t>(msg->angular.z * 100); // Scale the angular velocity
        steer_publisher_->publish(steer_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr steer_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToSteer>());
    rclcpp::shutdown();
    return 0;
}