#ifndef FRAME_H
#define FRAME_H

#include <Eigen/Dense>

#include "backend/factor/integration_base.h"

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
    backend::factor::IntegrationBase *pre_integration;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> linear_acceleration_buf;
    std::vector<Eigen::Vector3d> angular_velocity_buf;
};
#endif