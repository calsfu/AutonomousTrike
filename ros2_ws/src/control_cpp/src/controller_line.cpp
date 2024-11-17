#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "control_cpp/pid.h"
#include "common_utils/image_constants.hpp"

namespace 
{   
    // PID Gains, tune as needed
    constexpr double KP = 0.1;
    constexpr double KI = 0.01;
    constexpr double KD = 0.01;
}; // namespace

/// @brief ControllerLine class
/// @details This class is responsible for controlling the robot to follow a straight line. 
/// If we choose to use a modular pipeline with waypoints, this will be helpful for lateral control.
class ControllerLine : public rclcpp::Node
{
    public:
    ControllerLine() : Node("ControllerLine"), pid_(KP, KI, KD)
    {
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot_twist", 10);
        image_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("point_stamp", 10, std::bind(&ControllerLine::sub_callback, this, std::placeholders::_1));
    }

    private:
    void sub_callback(const geometry_msgs::msg::PointStamped& msg)
    {
        double error = msg.point.x -(ImageConstants::image_size_x / 2); // Calculate error as distance to midpoint

        double time_delay = msg.header.stamp.nanosec - prev_time_; // Calculate time delay
        prev_time_ = msg.header.stamp.nanosec; // Update previous time

        geometry_msgs::msg::Twist twist_msg; // Create Twist message
        twist_msg.angular.z = pid_.Update(error, time_delay); // Update PID controller
        twist_publisher_->publish(twist_msg); // Publish Twist message
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_; // Publisher of Twist messages
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr image_sub_; // Subscription to image messages
    PID pid_; // PID controller
    uint32_t prev_time_ = 0; // Previous time
};

int main()
{
    rclcpp::init(0, nullptr); // Initialize ROS
    rclcpp::spin(std::make_shared<ControllerLine>()); // Spin the node
    rclcpp::shutdown(); // Shutdown ROS
    return 0;
}