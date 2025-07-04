#include "common/frame.h"

namespace common {

void Frame::clear() {
    timestamp = 0.0;
    R = Eigen::Matrix3d::Identity();
    P = Eigen::Vector3d::Zero();
    V = Eigen::Vector3d::Zero();
    Ba = Eigen::Vector3d::Zero();
    Bg = Eigen::Vector3d::Zero();
    pre_integration.reset();

    dt_buf.clear();
    linear_acceleration_buf.clear();
    angular_velocity_buf.clear();
}

} // namespace common