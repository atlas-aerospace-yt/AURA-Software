#include "PID.h"

namespace pid {

pid::pid(float kp, float kd, float ki, float setpoint, float anti_wind_lim)
    : _kp(kp), _ki(ki), _kd(kd), _setpoint(setpoint), _anti_wind_lim(anti_wind_lim) {}

[[nodiscard]] auto pid::update(float state, float dt) -> float {
  // Compute the error term
  const float error = _setpoint - state;
  // Compute the derivative of the error term
  const float derivative = (error - _prev_error) / dt;

  // Compute the integral term
  _integral += error * dt;

  if (_anti_wind_lim != 0) {
    _integral = std::clamp(_integral, -_anti_wind_lim, +_anti_wind_lim);
  }

  // Update the previous error to be the current error
  _prev_error = error;

  return (_kp * error) + (_ki * _integral) + (_kd * derivative);
}

}  // namespace pid
