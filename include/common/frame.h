#ifndef COMMON__FRAME_H
#define COMMON__FRAME_H

#include <Eigen/Dense>
#include <memory>

#include "backend/factor/integration_base.h"

namespace common {

class Frame {
public:
    Frame() = default;

    void clear();

public:
    double timestamp;
    Eigen::Matrix3d R;
    Eigen::Vector3d P;
    Eigen::Vector3d V;
    Eigen::Vector3d Ba;
    Eigen::Vector3d Bg;
    std::unique_ptr<backend::factor::IntegrationBase> pre_integration;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> linear_acceleration_buf;
    std::vector<Eigen::Vector3d> angular_velocity_buf;
};

} // namespace common

#endif  // COMMON__FRAME_H