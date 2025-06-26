#include "backend/factor/pose_local_parameterization.h"

namespace backend {
namespace factor {

bool PoseLocalParameterization::Plus(const double *x, const double *delta,
                                     double *x_plus_delta) const {
  Eigen::Map<const Eigen::Vector3d> _p(x);
  Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

  Eigen::Map<const Eigen::Vector3d> dp(delta);

  Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

  Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
  Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

  p = _p + dp;
  q = (_q * dq).normalized();

  return true;
}
bool PoseLocalParameterization::PlusJacobian(const double *x, double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();

  return true;
}

bool PoseLocalParameterization::Minus(const double *y, const double *x, double *y_minus_x) const {
  Eigen::Map<const Eigen::Vector3d> p_x(x);
  Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);
  
  Eigen::Map<const Eigen::Vector3d> p_y(y);
  Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);

  Eigen::Map<Eigen::Vector3d> dp(y_minus_x);
  Eigen::Map<Eigen::Vector3d> dq(y_minus_x + 3);

  dp = p_y - p_x;
  
  Eigen::Quaterniond delta_q = q_x.inverse() * q_y;
  dq = 2.0 * delta_q.vec();
  if (delta_q.w() < 0) {
    dq = -dq;
  }

  return true;
}

bool PoseLocalParameterization::MinusJacobian(const double *x, double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
  j.topLeftCorner<3, 3>().setIdentity();
  j.topRightCorner<3, 4>().setZero();
  j.bottomLeftCorner<3, 3>().setZero();
  j.bottomRightCorner<3, 4>().setZero();
  j.block<3, 3>(3, 3).setIdentity();

  return true;
}

}  // namespace factor
}  // namespace backend
