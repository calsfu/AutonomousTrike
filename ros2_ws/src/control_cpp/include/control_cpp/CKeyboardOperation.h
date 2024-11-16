#pragma once

#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control_cpp/NRobotModel.h"

class CKeyboardOperation : public rclcpp::Node 
{
public: 
    CKeyboardOperation();

    virtual ~CKeyboardOperation();
private:
    void timerCallback();
    void processKey(char c);

    std::optional<char> readKey();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool isRunning_;
    std::shared_ptr<NRobotModel::KeysToVelocites> key_object_;
    struct termios old_term_, new_term_;
    double speed_linear_ = 0.0, speed_angular_ = 0.0;

};