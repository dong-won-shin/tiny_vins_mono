#ifndef BACKEND__FACTOR__PROJECTION_FACTOR_H
#define BACKEND__FACTOR__PROJECTION_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include "utility/utility.h"

namespace backend {
namespace factor {

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1> {
public:
    ProjectionFactor(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;
    void check(double** parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

}  // namespace factor
}  // namespace backend

#endif  // BACKEND__FACTOR__PROJECTION_FACTOR_H