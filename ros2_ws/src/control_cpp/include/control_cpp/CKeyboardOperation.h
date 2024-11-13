#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CKeyboardOperation : public rclcpp::Node 
{
public: 
    CKeyboardOperation();

private:
    void timerCallback();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    // std::shared_ptr<KeysToVelocities> key_object_;
    bool isRunning_;

};