#ifndef POSE_LOCAL_PARAMETERIZATION_H
#define POSE_LOCAL_PARAMETERIZATION_H

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>

#include "utility/utility.h"

namespace backend {
namespace factor {

class PoseLocalParameterization : public ceres::LocalParameterization {
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const {
        return 7;
    };
    virtual int LocalSize() const {
        return 6;
    };
};

}  // namespace factor
}  // namespace backend

#endif