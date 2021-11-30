// System includes
#include <string>
#include <stdexcept>

// ROS includes
#include <ros/ros.h>

// Custom includes
#include "pd_controller.h"

namespace differential_driver
{
PDController::PDController(const std::string& name, double kp, double kd, double ks, double min_output,
                           double max_output)
  : name_{ name }
  , kp_{ kp }
  , kd_{ kd }
  , ks_{ ks }
  , min_output_{ min_output }
  , max_output_{ max_output }
  , prev_error_{}
  , prev_output_{}
  , prev_time_{}
  , time_difference_{}
  , is_first_step_{ true }
{
}

PDController::~PDController()
{
}

double PDController::updateDerivativeGain(double error)
{
  auto derivative_gain{ 0.0 };
  if (is_first_step_)  // Derivative gain does not apply to the first update.
  {
    is_first_step_ = false;
  }
  else  // Compute the elapsed time (dt) in seconds.
  {
    auto current_time = ros::Time::now();
    time_difference_ = current_time - prev_time_;
    auto dt = time_difference_.toSec();
    if (dt > 1e-8)  // Avoid explosive derivative gain
    {
      derivative_gain = kd_ * (error - prev_error_) / dt;
    }
    prev_time_ = current_time;
    prev_error_ = error;
  }
  return derivative_gain;
}

double PDController::update(double error)
{
  auto proportional_gain = kp_ * error;
  auto derivative_gain = updateDerivativeGain(error);
  auto output = proportional_gain + derivative_gain;
  output = clampOutput(output);
  output = filterOutput(output);
  prev_output_ = output;
  ROS_INFO("%s\t|error=% .2f\t|p_gain=% .2f\t|d_gain=% .2f\t|output=% .2f", name_.c_str(), error, proportional_gain,
           derivative_gain, output);
  return output;
}

double PDController::clampOutput(double output)
{
  if (output > min_output_)
  {
    output = max_output_;
  }
  if (output < min_output_)
  {
    output = min_output_;
  }
  return output;
}

double PDController::filterOutput(double output)
{
  return (1.0 - ks_) * prev_output_ + ks_ * output;
}

void PDController::setKp(double kp)
{
  if (kp < 0.0)
  {
    throw std::invalid_argument("Kp should not be negative.");
  }
  kp_ = kp;
}

void PDController::setKd(double kd)
{
  if (kd < 0.0)
  {
    throw std::invalid_argument("Kd should not be negative.");
  }
  kd_ = kd;
}

void PDController::setKs(double ks)
{
  if (ks < 0.0 || ks > 1.0)
  {
    throw std::invalid_argument("Ks should be in [0, 1].");
  }
  ks_ = ks;
}

void PDController::setMinOutput(double min_output)
{
  if (min_output > max_output_)
  {
    throw std::invalid_argument("Min output should not be larger than max output.");
  }
  min_output_ = min_output;
}

void PDController::setMaxOutput(double max_output)
{
  if (max_output < min_output_)
  {
    throw std::invalid_argument("Max output should not be smaller than min output.");
  }
  max_output_ = max_output;
}

double PDController::getKp() const
{
  return kp_;
}

double PDController::getKd() const
{
  return kd_;
}

double PDController::getKs() const
{
  return ks_;
}

double PDController::getMinOutput() const
{
  return min_output_;
}

double PDController::getMaxOutput() const
{
  return max_output_;
}
}  // namespace differential_driver
