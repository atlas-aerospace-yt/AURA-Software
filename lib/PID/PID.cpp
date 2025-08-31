#include "PID.h"

namespace pid {

pid::pid(float kp, float ki, float kd, float setpoint, float anti_wind_lim)
    : _kp(kp),
      _ki(ki),
      _kd(kd),
      _setpoint(setpoint),
      _anti_wind_lim(anti_wind_lim) {}

auto pid::reset() -> void {
  _setpoint = 0;
  _integral = 0;
}

[[nodiscard]] auto pid::update(float state, float dt) -> float {
  // Compute the error term
  const float error = _setpoint - state;
  // Compute the derivative of the error term
  const float derivative = (state - _prev_state) / dt;

  // Compute the integral term
  if (_enabled_integral) {
    _integral += error * dt;

    if (_anti_wind_lim != 0) {
      _integral = std::clamp(_integral, -_anti_wind_lim, +_anti_wind_lim);
    }
  }

  // Update the previous error to be the current error
  _prev_state = error;

  return (_kp * error) + (_ki * _integral) + (_kd * derivative);
}

auto pid::enable_disable_interal(bool enabled) -> void {
  _integral = 0.0F;
  _enabled_integral = enabled;
}

}  // namespace pid
