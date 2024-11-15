#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

namespace NRobotModel 
{
    class KeysToVelocites;

    // Ackerman Steering Geometry



} // namespace NRobotModel

class NRobotModel::KeysToVelocites 
{
public:
    KeysToVelocites();

    /// \brief Update speeds based on key
    std::pair<double, double> UpdateSpeeds(char key);

private:
    double speed_linear_ = 0.0;
    double speed_angular_ = 0.0;
    double speed_delta_ = 0.1;
};