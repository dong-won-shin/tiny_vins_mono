#ifndef BACKEND__ESTIMATOR_H
#define BACKEND__ESTIMATOR_H

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <mutex>

#include "backend/optimizer.h"
#include "backend/sliding_window.h"
#include "common/common_types.h"
#include "common/image_frame.h"
#include "frontend/failure_detector.h"
#include "frontend/feature_manager.h"
#include "frontend/initialization/initializer.h"

namespace backend {

class Estimator {
public:
    Estimator();

    void setParameter();

    void processIMU(double dt, const Eigen::Vector3d& linear_acceleration, const Eigen::Vector3d& angular_velocity);
    void processImage(const common::ImageData& image, double timestamp);

    Eigen::Matrix3d r_ic_;
    Eigen::Vector3d t_ic_;

    SlidingWindow sliding_window_;

    common::SolverFlag solver_flag_;
    common::MarginalizationFlag marginalization_flag_;

    std::vector<Eigen::Vector3d> getSlidingWindowMapPoints() const;

private:
    void clearState();

    void slideWindow();
    void slideWindowNewGeneralFrame();
    void slideWindowOldKeyframe();

    void solveOdometry();

    void cleanupOldImageFrames(double timestamp);
    void cleanupPreIntegration(common::ImageFrame& frame);

    void propagateIMUState(int frame_index, double dt, const Eigen::Vector3d& linear_acceleration,
                           const Eigen::Vector3d& angular_velocity);
    void storeLastPoseInSlidingWindow();

    bool first_imu_;
    bool failure_occur_;

    double initial_timestamp_;
    Eigen::Vector3d prev_acc_, prev_gyro_;
    int frame_count_;

    Eigen::Vector3d g_;

    std::unique_ptr<backend::factor::IntegrationBase> tmp_pre_integration_;

    std::map<double, common::ImageFrame> all_image_frame_;

    Eigen::Matrix3d last_R_end_;
    Eigen::Vector3d last_P_end_;

    // Core components
    Optimizer optimizer_;
    frontend::FailureDetector failure_detector_;
    frontend::initialization::Initializer initializer_;
    frontend::FeatureManager feature_manager_;
    frontend::initialization::MotionEstimator motion_estimator_;

    mutable std::mutex estimator_mutex_;
};

}  // namespace backend

#endif  // BACKEND__ESTIMATOR_H
