#pragma once

#include <algorithm>

namespace pid {

//
// A PID class to use for control
//
class pid {
 private:
  const float _kp{};
  const float _ki{};
  const float _kd{};

  float _integral{};
  float _setpoint{};
  float _prev_state{};

  const float _anti_wind_lim;

  bool _enabled_integral;

 public:
  explicit pid(float kp, float ki, float kd, float setpoint = 0,
               float anti_wind_lim = 0);

  [[nodiscard]] auto kp() const -> float { return _kp; }
  [[nodiscard]] auto ki() const -> float { return _ki; }
  [[nodiscard]] auto kd() const -> float { return _kd; }

  [[nodiscard]] auto prev_error() const -> float { return _prev_state; }
  [[nodiscard]] auto integral() const -> float { return _integral; }
  [[nodiscard]] auto setpoint() const -> float { return _setpoint; }

  auto set_setpoint(float setpoint) -> void { _setpoint = setpoint; }

  //
  // Set all internal states back to 0
  //
  auto reset() -> void;

  //
  // Update the PID loop to get the control variable result
  //
  // @param state the current state e.g. AOA, displacement
  // @param dt the change in time between the last loop and now
  //
  // @returns float the control variable calculated by the controller
  //
  [[nodiscard]] auto update(float state, float dt) -> float;

  //
  // Enable or disable the integral to stop windup (also resets the integral)
  //
  // True is enabled, False is disabled
  //
  auto enable_disable_interal(bool enabled) -> void;
};

}  // namespace pid
