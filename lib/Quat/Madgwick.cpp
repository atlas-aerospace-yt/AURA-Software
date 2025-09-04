#include "Madgwick.h"

#include "utility.h"

namespace ori {

madgwick::madgwick(float beta, Vect ref_mag, Vect ref_acc)
    : _ref_mag(Quat(0.0F, ref_mag.x(), ref_mag.y(), ref_mag.z())),
      _ref_acc(Quat(0.0F, ref_acc.x(), ref_acc.y(), ref_acc.z())) {
  _beta = beta;
}

[[nodiscard]] void madgwick::madgwick::_combine_jacobian(
    float jacobian_acc[3][4], float jacobian_mag[3][4]) {
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 3; ++col) {
      _jacobian_t[row][col] = jacobian_acc[col][row];
    }
  }
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 3; ++col) {
      _jacobian_t[row][col + 3] = jacobian_mag[col][row];
    }
  }
}

[[nodiscard]] void madgwick::_jacobian(Quat q, Quat d, float jacobian[3][4]) {
  jacobian[0][0] = 2 * d.j() * q.k() - 2 * d.k() * q.j();
  jacobian[0][1] = 2 * d.j() * q.j() + 2 * d.k() * q.k();
  jacobian[0][2] = -4 * d.i() * q.j() + 2 * d.j() * q.i() - 2 * d.k() * q.w();
  jacobian[0][3] = -4 * d.i() * q.k() + 2 * d.k() * q.w() + 2 * d.k() * q.i();

  jacobian[1][0] = -2 * d.i() * q.k() + 2 * d.j() * q.i();
  jacobian[1][1] = 2 * d.i() * q.j() - 4 * d.j() * q.i() + 2 * d.k() * q.w();
  jacobian[1][2] = 2 * d.i() * q.j() + 2 * d.k() * q.k();
  jacobian[1][3] = -2 * d.i() * q.w() - 4 * d.j() * q.k() + 2 * q.k() * q.j();

  jacobian[2][0] = 2 * d.i() * q.j() - 2 * d.j() * q.i();
  jacobian[2][1] = 2 * d.i() * q.k() - 2 * d.j() * q.w() - 4 * d.k() * q.i();
  jacobian[2][2] = 2 * d.i() * q.w() + 2 * d.j() * q.k() - 4 * d.k() * q.j();
  jacobian[2][3] = 2 * d.i() * q.i() + 2 * d.j() * q.j();
}

[[nodiscard]] void madgwick::_combine_cost_func(Quat cost_acc, Quat cost_mag) {
  _cost[0] = cost_acc.i();
  _cost[1] = cost_acc.j();
  _cost[2] = cost_acc.k();
  _cost[3] = cost_mag.i();
  _cost[4] = cost_mag.j();
  _cost[5] = cost_mag.k();
}

[[nodiscard]] const Quat madgwick::_cost_func(Quat q, Quat d, Quat s) {
  return q.conjugate() * d * q - s;
}

[[nodiscard]] const Quat madgwick::_mult_jacobian_t_cost_func() {
  float grad[4];
  for (int i = 0; i < 4; ++i) {
    grad[i] = 0.0f;
    for (int j = 0; j < 6; ++j) {
      grad[i] += _jacobian_t[i][j] * _cost[j];
    }
  }
  return Quat(grad[0], grad[1], grad[2], grad[3]);
}

[[nodiscard]] const Quat madgwick::update(Quat ori, Vect gyro_v, Vect mag_v,
                                          Vect acc_v, float dt) {
  Quat gyro = Quat(0.0F, gyro_v.x(), gyro_v.y(), gyro_v.z());
  Quat acc = Quat(0.0F, acc_v.x(), acc_v.y(), acc_v.z());
  Quat mag = Quat(0.0F, mag_v.x(), mag_v.y(), mag_v.z());

  float jacobian_acc[3][4];
  float jacobian_mag[3][4];

  _jacobian(ori, _ref_acc, jacobian_acc);
  _jacobian(ori, _ref_mag, jacobian_mag);

  Quat cost_acc = _cost_func(ori, _ref_acc, acc);
  Quat cost_mag = _cost_func(ori, _ref_mag, mag);

  _combine_jacobian(jacobian_acc, jacobian_mag);
  _combine_cost_func(cost_acc, cost_mag);

  Quat grad = _mult_jacobian_t_cost_func();

  if (grad.mag() > 1e-8F) {
    grad = grad.normalise();
  }

  return ori + ((ori * gyro * 0.5F) - grad * _beta) * dt;
}

}  // namespace ori
