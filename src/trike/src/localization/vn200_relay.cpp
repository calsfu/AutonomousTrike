#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "vectornav/msg/Ins.msg"

using VNmsg = vectornav::msg::Ins;

class VN200Relay : public rclcpp::Node 
{
public:
    VN200Relay();

private:
    void odometry_callback(const VNmsg::SharedPtr msg) {
        auto odom_msg = nav_msgs::msg::Odometry();
        
        //Odometry contains pose and twist

        odom_msg.pose.pose.position.x = msg->latitude; 
        odom_msg.pose.pose.position.y = msg->longitude;
        odom_msg.pose.pose.position.z = msg->altitude; 

        // orientation we only need yaw
        double yaw_rad = msg->yaw * M_PI / 180.0; 
        odom_msg.pose.pose.orientation.w = cos(yaw_rad / 2.0);
        odom_msg.pose.pose.orientation.z = sin(yaw_rad / 2.0);
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;

        // Twist
        odom_msg.twist.twist.linear.x = msg->nedVelX;  // North velocity (m/s)
        odom_msg.twist.twist.linear.y = msg->nedVelY;  // East velocity (m/s)
        odom_msg.twist.twist.linear.z = msg->nedVelZ;  // Down velocity (m/s)

        publisher_->publish(odom_msg);

    }

    rclcpp::Subscription<Vectornav>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryRelayNode>());
    rclcpp::shutdown();
    return 0;
}