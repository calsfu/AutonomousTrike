#pragma once

// #include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

namespace NRobotModel 
{
    class KeysToVelocites 
    {
    public:
        KeysToVelocites() {}

        /// \brief Update speeds based on key
        std::pair<double, double> UpdateSpeeds(char key)
        {
            key = std::tolower(key);
            
            // Longitudinal Control
            if(key == 'w')
            {
                speed_linear_ = std::min(speed_linear_ + speed_delta_, 0.0); // Can't accelerate
            }
            else if(key == 's')
            {
                speed_linear_ = std::max(speed_linear_ - speed_delta_, -1.0);
            }
            else if(key == 'a') // Lateral Control
            {
                speed_angular_ = std::min(speed_angular_ + speed_delta_, 1.0);
            }
            else if(key == 'd')
            {
                speed_angular_ = std::max(speed_angular_ - speed_delta_, -1.0);
            }

            return {speed_linear_, speed_angular_};
        }

    private:
        double speed_linear_ = 0.0;
        double speed_angular_ = 0.0;
        double speed_delta_ = 0.1;
    };

    // Ackerman Steering Geometry



} // namespace NRobotModel

