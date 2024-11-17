#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CMotorCommand : public rclcpp::Node 
{
public:
    CMotorCommand() : Node("CMotorCommand")
    {
        // Get twist and 
        this->subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "robot_twist", 10, std::bind(&CMotorCommand::msgCallback, this));

    }

    void msgCallback()
    {

    }
    
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;

}; // class CMotorCommand