/**
 * Code from:
 * https://github.com/atlas-aerospace-yt/Arduino-IMU-Quaternions/tree/main
 */

#include "Quat.h"

#include <algorithm>
#include <cmath>

namespace ori {

[[nodiscard]] float Vect::mag() const {
  return sqrtf((x_ * x_) + (y_ * y_) + (z_ * z_));
}

[[nodiscard]] Vect Vect::to_radians() const {
  return Vect(x_ * deg_to_rad, y_ * deg_to_rad, z_ * deg_to_rad);
}

[[nodiscard]] Vect Vect::to_degrees() const {
  return Vect(x_ * rad_to_deg, y_ * rad_to_deg, z_ * rad_to_deg);
}

[[nodiscard]] Quat Vect::to_quat() const {
  const float cr = cosf(x_ * 0.5f);
  const float sr = sinf(x_ * 0.5f);
  const float cp = cosf(y_ * 0.5f);
  const float sp = sinf(y_ * 0.5f);
  const float cy = cosf(z_ * 0.5f);
  const float sy = sinf(z_ * 0.5f);

  return Quat(cr * cp * cy - sr * sp * sy, sr * cp * cy + cr * sp * sy,
              cr * sp * cy - sr * cp * sy, cr * cp * sy + sr * sp * cy);
}

// Euler angles from quaternions
[[nodiscard]] Vect Quat::to_euler() const {
  const float x1 = (2.0F * j_ * k_) - (2.0F * w_ * i_);
  const float x2 = (2.0F * w_ * w_) + (2.0F * k_ * k_) - 1.0F;
  const float x_ypr = atan2(x1, x2);

  const float y_val = (2.0F * i_ * k_) + (2.0F * w_ * j_);
  const float clamped_y_val = std::clamp(y_val, -1.0F, 1.0F);
  const float y_ypr = -asin(clamped_y_val);

  const float z1 = (2.0F * i_ * j_) - (2.0F * w_ * k_);
  const float z2 = (2.0F * w_ * w_) + (2.0F * i_ * i_) - 1.0F;
  const float z_ypr = atan2(z1, z2);

  return Vect(-x_ypr, -y_ypr, -z_ypr);
}

[[nodiscard]] Quat Quat::update(Vect& v, float dt) const {
  const Quat omega(0.0F, v.x(), v.y(), v.z());
  const Quat q_dot = (*this) * 0.5F * omega;
  return *this + (q_dot * dt);
}

[[nodiscard]] Quat Quat::conjugate() const { return Quat(w_, -i_, -j_, -k_); }

[[nodiscard]] Quat Quat::normalise() const {
  const float mag = sqrtf((w_ * w_) + (i_ * i_) + (j_ * j_) + (k_ * k_));
  return Quat(w_ / mag, i_ / mag, j_ / mag, k_ / mag);
}

[[nodiscard]] Vect Quat::calc_error_axis(Quat desired) const {
  const Quat err_quat = desired * this->conjugate();

  if (err_quat.w() < 0.0F) {
    return Vect(-err_quat.i(), -err_quat.j(), -err_quat.k());
  }
  return Vect(err_quat.i(), err_quat.j(), err_quat.k());
}

// This function is slower than calc_error_axes only use if exact
// angles are required
[[nodiscard]] Vect Quat::calc_error(Quat desired) const {
  const Quat err_quat = desired * this->conjugate();

  // Ensure shortest path is followed
  Vect error_axes{};
  if (err_quat.w() < 0.0F) {
    error_axes = Vect(-err_quat.i(), -err_quat.j(), -err_quat.k());
  } else {
    error_axes = Vect(err_quat.i(), err_quat.j(), err_quat.k());
  }

  const float magnitude = error_axes.mag();

  // Avoid errors near 0
  if (magnitude < 1e-6f) {
    return Vect(0.0f, 0.0f, 0.0f);
  }

  const float angle = 2.0f * std::atan2(magnitude, err_quat.w());

  return error_axes * (angle / magnitude);
}

}  // namespace ori
