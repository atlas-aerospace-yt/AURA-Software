/**
 * Code from:
 * https://github.com/atlas-aerospace-yt/Arduino-IMU-Quaternions/tree/main
 */

#include "Quat.h"

#include <algorithm>
#include <cmath>

namespace ori {

// Euler angles from quaternions
[[nodiscard]] Vect Quat::to_euler() const {
  const float x1 = (2.0F * i_ * j_) - (2.0F * w_ * k_);
  const float x2 = (2.0F * w_ * w_) + (2.0F * i_ * i_) - 1.0F;
  const float x_ypr = atan2(x1, x2);

  const float y_val = (2.0F * i_ * k_) + (2.0F * w_ * j_);
  const float clamped_y_val = std::clamp(y_val, -1.0F, 1.0F);
  const float y_ypr = -asin(clamped_y_val);

  const float z1 = (2.0F * j_ * k_) - (2.0F * w_ * i_);
  const float z2 = (2.0F * w_ * w_) + (2.0F * k_ * k_) - 1.0F;
  const float z_ypr = atan2(z1, z2);

  return Vect(x_ypr, y_ypr, z_ypr);
}

[[nodiscard]] Vect Vect::to_radians() const {
  return Vect(x_ * deg_to_rad, y_ * deg_to_rad, z_ * deg_to_rad);
}

[[nodiscard]] Vect Vect::to_degrees() const {
  return Vect(x_ * rad_to_deg, y_ * rad_to_deg, z_ * rad_to_deg);
}

[[nodiscard]] Quat Quat::update(Vect& v, float dt) const {
  const Quat omega(0.0F, v.x(), v.y(), v.z());
  const Quat q_dot = (*this) * 0.5F * omega;
  return *this + (q_dot * dt);
}

}  // namespace ori
