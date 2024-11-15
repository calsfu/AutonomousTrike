#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CMotorCommand : public rclcpp::Node 
{
public:
    CMotorCommand();

    void msgCallback();
    
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;


};