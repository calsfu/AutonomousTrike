#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "trike/constants.hpp"

class DataManager : public rclcpp::Node 
{
public:
    DataManager() : Node("data_manager")
    {
        brake_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
            "control/brake", 10, std::bind(&DataManager::brake_callback, this, std::placeholders::_1)
        );
        steer_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
            "control/steer", 10, std::bind(&DataManager::steer_callback, this, std::placeholders::_1)
        );
        mode_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
            "mode", 10, std::bind(&DataManager::mode_callback, this, std::placeholders::_1)
        );
        brake_publisher_ = this->create_publisher<std_msgs::msg::Int8>("control/brake/new", 10);
        steer_publisher_ = this->create_publisher<std_msgs::msg::Int8>("control/steer/new", 10);
    }
private:
    void brake_callback(const std_msgs::msg::Int8::SharedPtr msg) {
        auto new_message = std_msgs::msg::Int8();
        new_message.data = msg->data;
        switch (mode_) {
            case trike::PARK:
                // Always publish 1 when in PARK mode
                new_message.data = 1;
                brake_publisher_->publish(new_message);
                break;
            case trike::NEUTRAL:
                // Always publish 0 when in NEUTRAL mode
                new_message.data = 0;
                brake_publisher_->publish(new_message);
                break;
            case trike::MANUAL:
                // Publish the received value when in MANUAL mode
                brake_publisher_->publish(new_message);
                break;
            case trike::AUTONOMOUS:
                // Publish the received value when in AUTONOMOUS mode
                brake_publisher_->publish(new_message);
                break;
        }
    }

    void steer_callback(const std_msgs::msg::Int8::SharedPtr msg) {
        auto new_message = std_msgs::msg::Int8();
        new_message.data = msg->data;
        switch (mode_) {
            case trike::PARK:
                // Always publish 0 when in PARK mode
                new_message.data = 0;
                steer_publisher_->publish(new_message);
                break;
            case trike::NEUTRAL:
                // Always publish 0 when in NEUTRAL mode
                new_message.data = 0;
                steer_publisher_->publish(new_message);
                break;
            case trike::MANUAL:
                // Publish the received value when in MANUAL mode
                steer_publisher_->publish(new_message);
                break;
            case trike::AUTONOMOUS:
                // Publish the received value when in AUTONOMOUS mode
                steer_publisher_->publish(new_message);
                break;
        }
    }

    void mode_callback(const std_msgs::msg::Int8::SharedPtr msg) {
        mode_ = msg->data;
        auto new_message = std_msgs::msg::Int8();
        switch (mode_) {
            case trike::PARK:
                // Switch to brake
                new_message.data = 1;
                brake_publisher_->publish(new_message);
                break;
            default:
                // unbrake
                new_message.data = 0;
                steer_publisher_->publish(new_message);
                break;
        }
    }

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr brake_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr steer_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_subscription_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr brake_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr steer_publisher_;
    int mode_ = trike::PARK;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataManager>());
    rclcpp::shutdown();
    return 0;
}