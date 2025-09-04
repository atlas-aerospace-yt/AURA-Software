/**
 * Code from:
 * https://github.com/atlas-aerospace-yt/Arduino-IMU-Quaternions/tree/main
 */

#pragma once

#include "math.h"

namespace ori {
class Quat;
class Vect;

constexpr float PI = 3.14159F;
constexpr float deg_to_rad = PI / 180.0F;
constexpr float rad_to_deg = 180.0F / PI;

class Vect {
 private:
  float x_{}, y_{}, z_{};

 public:
  constexpr Vect() = default;
  constexpr Vect(float x, float y, float z) : x_(x), y_(y), z_(z) {}

  [[nodiscard]] float x() const { return x_; }
  [[nodiscard]] float y() const { return y_; }
  [[nodiscard]] float z() const { return z_; }

  [[nodiscard]] Vect operator/(float f) const {
    return Vect(x_ / f, y_ / f, z_ / f);
  }

  [[nodiscard]] Vect operator*(float f) const {
    return Vect(x_ * f, y_ * f, z_ * f);
  }

  [[nodiscard]] Vect operator+(Vect v) const {
    return Vect(x_ + v.x(), y_ + v.y(), z_ + v.z());
  }

  [[nodiscard]] Vect operator-(Vect v) const {
    return Vect(x_ - v.x(), y_ - v.y(), z_ - v.z());
  }

  // Convert from degrees to radians
  [[nodiscard]] Vect to_radians() const;

  // Convert from radians to degrees
  [[nodiscard]] Vect to_degrees() const;

  // Calculate magnitude of vector
  [[nodiscard]] float mag() const;

  // Calculate unit vector
  [[nodiscard]] Vect normalise() const;

  // Calculate quaternion from euler
  [[nodiscard]] Quat to_quat() const;

  // Calculate the dot product
  [[nodiscard]] static float dot(Vect a, Vect b);

  // Calculate the cross product
  [[nodiscard]] static Vect cross(Vect a, Vect b);

  // Calculate the quaternions from two vectors
  [[nodiscard]] static Quat quat_from_two_vect(Vect a, Vect b);
};

class Quat {
 private:
  float w_{}, i_{}, j_{}, k_{};

 public:
  constexpr Quat() = default;
  constexpr Quat(float w, float i, float j, float k)
      : w_(w), i_(i), j_(j), k_(k) {}

  [[nodiscard]] float w() const { return w_; }
  [[nodiscard]] float i() const { return i_; }
  [[nodiscard]] float j() const { return j_; }
  [[nodiscard]] float k() const { return k_; }

  [[nodiscard]] Quat operator/(float f) const {
    return Quat(w_ / f, i_ / f, j_ / f, k_ / f);
  }

  [[nodiscard]] Quat operator*(const Quat& q) const {
    return Quat((w_ * q.w()) - (i_ * q.i()) - (j_ * q.j()) - (k_ * q.k()),
                (w_ * q.i()) + (i_ * q.w()) + (j_ * q.k()) - (k_ * q.j()),
                (w_ * q.j()) - (i_ * q.k()) + (j_ * q.w()) + (k_ * q.i()),
                (w_ * q.k()) + (i_ * q.j()) - (j_ * q.i()) + (k_ * q.w()));
  }

  [[nodiscard]] Quat operator*(float f) const {
    return Quat(w_ * f, i_ * f, j_ * f, k_ * f);
  }

  [[nodiscard]] Quat operator+(const Quat& q) const {
    return Quat(w_ + q.w(), i_ + q.i(), j_ + q.j(), k_ + q.k());
  }

  [[nodiscard]] Quat operator-(const Quat& q) const {
    return Quat(w_ - q.w(), i_ - q.i(), j_ - q.j(), k_ - q.k());
  }

  [[nodiscard]] Quat operator+(float f) const {
    return Quat(w_ + f, i_ + f, j_ + f, k_ + f);
  }

  [[nodiscard]] Quat operator-(float f) const {
    return Quat(w_ - f, i_ - f, j_ - f, k_ - f);
  }

  [[nodiscard]] Quat conjugate() const;

  [[nodiscard]] float mag() const;

  [[nodiscard]] Quat normalise() const;

  // Quaternions from angular rate using madgwick paper
  [[nodiscard]] Quat update(Vect& v, float dt) const;

  // Calculate the equivalent euler angles
  [[nodiscard]] Vect to_euler() const;

  //
  // Calculate the rotation error axes (x y z)
  //
  // @param desired the desired quaternion rotation
  //
  [[nodiscard]] Vect calc_error_axis(Quat desired) const;

  //
  // Calculate the rotation error (x y z)
  //
  // This function is slower than calc_error_axes only use if exact
  // angles are required
  //
  // @param desired the desired quaternion rotation
  //
  [[nodiscard]] Vect calc_error(Quat desired) const;
};

}  // namespace ori
