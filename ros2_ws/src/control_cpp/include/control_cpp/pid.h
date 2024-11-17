#pragma once

/// @brief PID Controller class
class PID 
{
public:
    /// @brief Constructor
    /// @param kp Propotional Gain
    /// @param ki Integral Gain
    /// @param kd Derivative Gain
    PID(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd) {}

    /// @brief Update the PID controller
    /// @param error Error value
    /// @param dt Time difference    
    double Update(double error, double dt)
    {
        double update_val = proportional(error) + integral(error, dt) + derivative(error, dt);
        prev_error_ = error;
        return update_val;
    }

private:
    /// @brief proportional control
    /// @param error error value
    /// @return proportional control value
    double proportional(double error)
    {
        return kp_ * error;
    }

    /// @brief perform integral control 
    /// @param error error value
    /// @param dt timestep
    /// @return integral control value
    double integral(double error, double dt)
    {
        integral_ += error * dt;
        return ki_ * integral_;
    }

    /// @brief perform derivative control
    /// @param error error value
    /// @param dt timestep
    /// @return derivative control value
    double derivative(double error, double dt)
    {
        if(prev_error_ == 0)
        {
            return 0;
        }
        return kd_ * (error - prev_error_) / dt;   
    }

    double kp_, ki_, kd_; // PID gains
    double integral_ = 0.0, prev_error_ = 0.0; // Integral and previous error values

};