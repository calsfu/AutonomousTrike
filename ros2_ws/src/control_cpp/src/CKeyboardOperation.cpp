// Public Includes
#include <chrono>
#include <iostream>
#include <memory>
#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control_cpp/NRobotModel.h"


namespace
{
    char getch() {
        struct termios old_term, new_term;
        char ch;
        tcgetattr(STDIN_FILENO, &old_term);
        new_term = old_term;
        new_term.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_term);
        ch = getchar(); // Blocking, but simpler
        tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
        return ch;
    }

}

class CKeyboardOperation : public rclcpp::Node 
{
public: 
    CKeyboardOperation() : Node("CKeyboardOperation"), isRunning_(true)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot_twist", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&CKeyboardOperation::timerCallback, this));
        key_object_ = std::make_shared<NRobotModel::KeysToVelocites>();

        // Setup Keyboard
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

        RCLCPP_INFO(this->get_logger(), "Keyboard Operation Node has been started");
    }


    virtual ~CKeyboardOperation()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
    }
    
private:
    void timerCallback() 
    {
        // Add get key here
        auto key = readKey();
        if(key) {
            processKey(*key);
        }

        geometry_msgs::msg::Twist msg;
        msg.linear.x = speed_linear_;
        msg.angular.z = speed_angular_;

        publisher_->publish(msg);
    }

    void processKey(char key)
    {
        if(key == 'q') 
        {
            RCLCPP_INFO(this->get_logger(), "Shutdown intiated by %s", this->get_name());
            rclcpp::shutdown();
            isRunning_ = false;
        }
        else 
        {
            std::tie(speed_linear_, speed_angular_) = key_object_->UpdateSpeeds(key);
        }
    }

    std::optional<char> readKey()
    {
        return getch();
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool isRunning_;
    std::shared_ptr<NRobotModel::KeysToVelocites> key_object_;
    struct termios old_term_, new_term_;
    double speed_linear_ = 0.0, speed_angular_ = 0.0;

}; // class CKeyboardOperation

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CKeyboardOperation>());
    rclcpp::shutdown();
    return 0;
}