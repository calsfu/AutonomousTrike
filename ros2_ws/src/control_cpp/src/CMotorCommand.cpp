// Self Include
#include "control_cpp/CMotorCommand.h"



CMotorCommand::CMotorCommand() : Node("CMotorCommand")
{
    // Get twist and 
    this->subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "robot_twist", 10, std::bind(&CMotorCommand::msgCallback, this));

}

void CMotorCommand::msgCallback() 
{

}