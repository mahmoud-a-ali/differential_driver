#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H

#include <ros/ros.h>
#include <string>

namespace differential_driver
{
class PDController
{
public:
  /// @brief Constructor
  ///
  /// @param name Controller name, e.g., linear/angular.
  /// @param kp Proportional gain coefficient.
  /// @param kd Derivative gain coefficient.
  /// @param ks Smoothing parameter in [0, 1]. Larger value means that the output changes faster.
  /// @param min_output Min of output.
  /// @param max_output Max of output.
  PDController(const std::string& name, double kp, double kd, double ks, double min_output, double max_output);

  /// @brief Deconstructor.
  virtual ~PDController();

  // Disable copy constructor.
  PDController(const PDController&) = delete;

  // Disable assignment constructor.
  PDController& operator=(const PDController&) = delete;

  ///////////////
  // Functions //
  ///////////////

  /// @brief Update the controller output based on the error.
  ///
  /// @param error Control error.
  ///
  /// @return Control output.
  double update(double error);

  /// @brief Clamp the control output to be in the range [min_output, max_output].
  ///
  /// @param output Control output before clampping.
  ///
  /// @return Control output after clampping.
  double clampOutput(double output);

  /// @brief Filter the control output: (1 - Ks) * prev_output + Ks * output.
  ///
  /// @param output Control output before filtering.
  ///
  /// @return Control output after filtering.
  double filterOutput(double output);

  /////////////
  // Setters //
  /////////////

  /// @brief Set the proportional gain coefficient.
  ///
  /// @param kp Proportional gain coefficient.
  void setKp(double kp);

  /// @brief Set the derivative gain coefficient.
  ///
  /// @param kd Derivative gain coefficient.
  void setKd(double kd);

  /// @brief Set the smoothing parameter.
  ///
  /// @param ks Smoothing parameter [0, 1].
  void setKs(double ks);

  /// @brief Set the min of control output.
  ///
  /// @param min_output Min of control output.
  void setMinOutput(double min_output);

  /// @brief Set the max of control output.
  ///
  /// @param max_output Max of control output.
  void setMaxOutput(double max_output);

  /////////////
  // Getters //
  /////////////

  /// @brief Get the proportional gain coefficient.
  ///
  /// @return Proportional gain coefficient.
  double getKp() const;

  /// @brief Get the derivative gain coefficient.
  ///
  /// @return Derivative gain coefficient.
  double getKd() const;

  /// @brief Get the smoothing parameter.
  ///
  /// @return Smoothing parameter.
  double getKs() const;

  /// @brief Get the min of control output.
  ///
  /// @return Min of control output.
  double getMinOutput() const;

  /// @brief Get the max of control output.
  ///
  /// @return Max of control output.
  double getMaxOutput() const;

private:
  /// @brief Update derivative gain.
  ///
  /// @param error Control error.
  ///
  /// @return Derivative gain.
  double updateDerivativeGain(double error);

  /// @brief Name of this controller, e.g. linear or angular velocity.
  std::string name_;

  /// @brief Proportional gain coefficient.
  double kp_;

  /// @brief Derivative gain coefficient.
  double kd_;

  /// @brief Smoothing parameter in [0, 1]. Larger value means that the output changes faster.
  double ks_;

  /// @brief Min of output.
  double min_output_;

  /// @brief Max of output.
  double max_output_;

  /// @brief Error in the previous step.
  double prev_error_;

  /// @brief Last output.
  double prev_output_;

  /// @brief Timestamp of the last step.
  ros::Time prev_time_;

  /// @brief Time elapsed between two consecutive steps.
  ros::Duration time_difference_;

  /// @brief Is this the first update?
  bool is_first_step_;
};
}  // namespace differential_driver

#endif /* end of include guard: PD_CONTROLLER_H */
