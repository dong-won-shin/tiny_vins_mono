#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>

#include "backend/factor/integration_base.h"
#include "backend/optimizer.h"
#include "backend/sliding_window.h"
#include "common/common_types.h"
#include "common/frame.h"
#include "common/image_frame.h"
#include "frontend/failure_detector.h"
#include "frontend/feature_manager.h"
#include "frontend/initialization/initializer.h"
#include "frontend/initialization/solve_5pts.h"

namespace backend {

class Estimator {
public:
    Estimator();

    void setParameter();

    void processIMU(double dt, const Eigen::Vector3d& linear_acceleration, const Eigen::Vector3d& angular_velocity);
    void processImage(const ImageData& image, double timestamp);

    Matrix3d r_ic_;
    Vector3d t_ic_;

    SlidingWindow sliding_window_;

    SolverFlag solver_flag_;
    MarginalizationFlag marginalization_flag_;

    std::vector<Eigen::Vector3d> getSlidingWindowMapPoints() const;

    mutable std::mutex estimator_mutex_;

private:
    void clearState();

    void slideWindow();
    void slideWindowNewGeneralFrame();
    void slideWindowOldKeyframe();

    void solveOdometry();

    void cleanupOldImageFrames(double timestamp);
    void cleanupPreIntegration(ImageFrame& frame);

    void propagateIMUState(int frame_index, double dt, const Vector3d& linear_acceleration,
                           const Vector3d& angular_velocity);
    void storeLastPoseInSlidingWindow();

    bool first_imu_;
    bool failure_occur_;

    double initial_timestamp_;
    Vector3d prev_acc_, prev_gyro_;
    int frame_count_;

    Vector3d g_;

    IntegrationBase* tmp_pre_integration_;

    std::map<double, ImageFrame> all_image_frame_;

    FeatureManager feature_manager_;
    MotionEstimator motion_estimator_;

    Matrix3d last_R_end_;
    Vector3d last_P_end_;

    // Core components
    Optimizer optimizer_;
    FailureDetector failure_detector_;
    Initializer initializer_;
};

} // namespace backend

#endif  // ESTIMATOR_H
