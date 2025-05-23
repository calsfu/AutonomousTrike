#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "trike/constants.hpp"
#include <boost/asio.hpp>
#include <string>

class SteeringController : public rclcpp::Node 
{
public:
    SteeringController() : Node("steering_controller"), serial_port_(io_)
    {  
        subscription_ = this->create_subscription<std_msgs::msg::Int8>(
            "control/steer/new", 10, std::bind(&SteeringController::steering_callback, this, std::placeholders::_1)
        );        
        audio_publisher_ = this->create_publisher<std_msgs::msg::Int8>("audio_command", 10);

        try {
            // open the serial port explicitly
            serial_port_.open(port_); 
            RCLCPP_INFO(this->get_logger(), "Successfully opened serial port: %s", port_.c_str());

            // set baud rate to 115200
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(9600));
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            // rclcpp::shutdown();
        }


        unsigned char byte = 0;
        boost::asio::write(serial_port_, boost::asio::buffer(&byte, sizeof(byte)));

    }

    void steering_callback(const std_msgs::msg::Int8::SharedPtr msg) {
        if (serial_port_.is_open()) {
            signed char byte = msg->data;
            std_msgs::msg::Int8 audio_msg;
            byte *= 127;
            boost::asio::write(serial_port_, boost::asio::buffer(&byte, sizeof(byte)));
            RCLCPP_INFO(this->get_logger(), "Sent: %d", byte);
            if(byte == 127) {
                audio_msg.data = trike::TURNING_LEFT;
                audio_publisher_->publish(audio_msg);
            } else {
                audio_msg.data = trike::TURNING_RIGHT;
                audio_publisher_->publish(audio_msg); 
            }
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr audio_publisher_;
    boost::asio::io_service io_;
    boost::asio::serial_port serial_port_;
    const std::string port_ = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_85935333337351507061-if00";
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteeringController>());
    rclcpp::shutdown();
    return 0;
}