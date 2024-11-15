
// Self Include
#include "control_cpp/NRobotModel.h"

namespace NRobotModel 
{
    class KeysToVelocites;
} // namespace NRobWotModel

std::pair<double, double> NRobotModel::KeysToVelocites::UpdateSpeeds(char key)
{
    key = std::tolower(key);
    
    // Longitudinal Control
    if(key == 's')
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