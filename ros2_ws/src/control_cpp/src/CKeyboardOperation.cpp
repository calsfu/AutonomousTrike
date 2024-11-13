// Public Includes
#include <chrono>
#include <iostream>
#include <memory>

// Self Include
#include "control_cpp/CKeyboardOperation.h"

CKeyboardOperation::CKeyboardOperation() : Node("CKeyboardOperation"), isRunning_(true)
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot_twist", 10);
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&CKeyboardOperation::timerCallback, this));
}
void CKeyboardOperation::timerCallback() 
{
    char key = 't';

    if(key == 'q') 
    {
        RCLCPP_INFO(this->get_logger(), "Shutdown intiated by %s", this->get_name());
        rclcpp::shutdown();
        isRunning_ = false;
    }
    else 
    {
        double speed_linear = 0.0, speed_angular = 0.0;
        // std::tie(speed_linear, speed_angular) = key_object->updateSpeeds(key);

        geometry_msgs::msg::Twist msg;
        msg.linear.x = speed_linear;
        msg.angular.z = speed_angular;

        publisher_->publish(msg);
    }
}

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CKeyboardOperation>());
    rclcpp::shutdown();
    return 0;
}