#include <fstream>
#include <iostream>
#include <mutex>

#include "backend/estimator.h"
#include "utility/config.h"

namespace backend {

Estimator::Estimator()
    : feature_manager_(),
      optimizer_(&sliding_window_, &feature_manager_),
      failure_detector_(&sliding_window_, &feature_manager_),
      initializer_(&sliding_window_, &feature_manager_, &motion_estimator_, &all_image_frame_, &frame_count_,
                   &marginalization_flag_, &g_, &r_ic_, &t_ic_) {
    clearState();
}

void Estimator::setParameter() {
    t_ic_ = utility::g_config.camera.t_ic;
    r_ic_ = utility::g_config.camera.r_ic;
    backend::factor::ProjectionFactor::sqrt_info = (utility::g_config.camera.focal_length / 1.5) * Eigen::Matrix2d::Identity();

    // Set extrinsic parameters in optimizer
    optimizer_.setExtrinsicParameters(t_ic_, r_ic_);
}

void Estimator::clearState() {
    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        sliding_window_.clearSlidingWindow();
    }

    t_ic_ = Eigen::Vector3d::Zero();
    r_ic_ = Eigen::Matrix3d::Identity();

    all_image_frame_.clear();

    solver_flag_ = common::SolverFlag::INITIAL;
    first_imu_ = false;
    frame_count_ = 0;
    initial_timestamp_ = 0;

    tmp_pre_integration_.reset();

    feature_manager_.clearState();

    failure_occur_ = 0;
}

void Estimator::propagateIMUState(int frame_index, double dt, const Eigen::Vector3d& linear_acceleration,
                                  const Eigen::Vector3d& angular_velocity) {
    // Store IMU data in buffers
    sliding_window_.pushBackBuffer(frame_index, dt, linear_acceleration, angular_velocity);

    // Bias-corrected measurements
    Eigen::Vector3d bias_corrected_prev_acc =
        sliding_window_[frame_index].R * (prev_acc_ - sliding_window_[frame_index].Ba) - g_;
    Eigen::Vector3d bias_corrected_gyro = 0.5 * (prev_gyro_ + angular_velocity) - sliding_window_[frame_index].Bg;

    // update rotation: R = R * exp(gyro*dt)
    sliding_window_[frame_index].R *= Utility::deltaQ(bias_corrected_gyro * dt).toRotationMatrix();

    // Propagate position and velocity using trapezoidal integration
    Eigen::Vector3d bias_corrected_curr_acc =
        sliding_window_[frame_index].R * (linear_acceleration - sliding_window_[frame_index].Ba) - g_;
    Eigen::Vector3d bias_corrected_acc = 0.5 * (bias_corrected_prev_acc + bias_corrected_curr_acc);

    // Update position: P = P + V*dt + 0.5*a*dt^2
    // Update velocity: V = V + a*dt
    sliding_window_[frame_index].P += dt * sliding_window_[frame_index].V + 0.5 * dt * dt * bias_corrected_acc;
    sliding_window_[frame_index].V += dt * bias_corrected_acc;
}

void Estimator::processIMU(double dt, const Eigen::Vector3d& linear_acceleration,
                           const Eigen::Vector3d& angular_velocity) {
    std::lock_guard<std::mutex> lock(estimator_mutex_);
    // check initial IMU data
    if (!first_imu_) {
        first_imu_ = true;
        prev_acc_ = linear_acceleration;
        prev_gyro_ = angular_velocity;
    }

    // if pre_integrations[frame_count_] is not initialized, initialize it
    // this is generated at every new image frame
    if (!sliding_window_[frame_count_].pre_integration) {
        sliding_window_[frame_count_].pre_integration = std::make_unique<backend::factor::IntegrationBase>(
            prev_acc_, prev_gyro_, sliding_window_[frame_count_].Ba, sliding_window_[frame_count_].Bg);
    }

    if (frame_count_ != 0) {
        sliding_window_.pushBackPreintegration(frame_count_, dt, linear_acceleration, angular_velocity);
        tmp_pre_integration_->push_back(dt, linear_acceleration, angular_velocity);
        propagateIMUState(frame_count_, dt, linear_acceleration, angular_velocity);
    }

    prev_acc_ = linear_acceleration;
    prev_gyro_ = angular_velocity;
}

void Estimator::processImage(const common::ImageData& image, double timestamp) {
    std::lock_guard<std::mutex> lock(estimator_mutex_);
    if (feature_manager_.addFeatureAndCheckParallax(frame_count_, image)) {
        marginalization_flag_ = common::MarginalizationFlag::MARGIN_OLD_KEYFRAME;
    } else {
        marginalization_flag_ = common::MarginalizationFlag::MARGIN_NEW_GENERAL_FRAME;
    }

    sliding_window_[frame_count_].timestamp = timestamp;

    common::ImageFrame imageframe(image, timestamp);
    imageframe.pre_integration = std::move(tmp_pre_integration_);
    all_image_frame_.insert(std::make_pair(timestamp, std::move(imageframe)));
    tmp_pre_integration_ = std::make_unique<backend::factor::IntegrationBase>(prev_acc_, prev_gyro_, sliding_window_[frame_count_].Ba,
                                                                sliding_window_[frame_count_].Bg);

    if (solver_flag_ == common::SolverFlag::INITIAL) {
        if (frame_count_ == WINDOW_SIZE) {
            bool visual_map_init_result = false;
            if (timestamp - initial_timestamp_ > 0.1) {
                visual_map_init_result = initializer_.initialize();
                initial_timestamp_ = timestamp;
            }

            if (visual_map_init_result) {
                solver_flag_ = common::SolverFlag::NON_LINEAR;
                solveOdometry();
                slideWindow();
                feature_manager_.removeFailures();

                storeLastPoseInSlidingWindow();
            } else {
                slideWindow();
            }
        } else {
            frame_count_++;
        }
    } else if (solver_flag_ == common::SolverFlag::NON_LINEAR) {
        solveOdometry();
        slideWindow();
        feature_manager_.removeFailures();

        storeLastPoseInSlidingWindow();
    }
}

void Estimator::storeLastPoseInSlidingWindow() {
    last_R_end_ = sliding_window_.back().R;
    last_P_end_ = sliding_window_.back().P;
}

void Estimator::cleanupOldImageFrames(double timestamp) {
    auto it_0 = all_image_frame_.find(timestamp);
    if (it_0 == all_image_frame_.end()) {
        return;
    }

    // 1. Cleanup pre_integration of current frame
    cleanupPreIntegration(it_0->second);

    // 2. Cleanup pre_integration of previous frames
    for (auto it = all_image_frame_.begin(); it != it_0; ++it) {
        cleanupPreIntegration(it->second);
    }

    // 3. Remove frames
    all_image_frame_.erase(all_image_frame_.begin(), it_0);
    all_image_frame_.erase(timestamp);
}

void Estimator::cleanupPreIntegration(common::ImageFrame& frame) {
    frame.pre_integration.reset();
}

void Estimator::slideWindow() {
    if (frame_count_ == WINDOW_SIZE) {
        if (marginalization_flag_ == common::MarginalizationFlag::MARGIN_OLD_KEYFRAME) {
            slideWindowOldKeyframe();
        } else if (marginalization_flag_ == common::MarginalizationFlag::MARGIN_NEW_GENERAL_FRAME) {
            slideWindowNewGeneralFrame();
        }
    }
}

void Estimator::slideWindowNewGeneralFrame() {
    // preintegrate new general frame onto the last keyframe
    for (unsigned int i = 0; i < sliding_window_[WINDOW_SIZE].dt_buf.size(); i++) {
        double tmp_dt = sliding_window_[WINDOW_SIZE].dt_buf[i];
        Vector3d tmp_linear_acceleration = sliding_window_[WINDOW_SIZE].linear_acceleration_buf[i];
        Vector3d tmp_angular_velocity = sliding_window_[WINDOW_SIZE].angular_velocity_buf[i];

        sliding_window_.pushBackPreintegration(WINDOW_SIZE - 1, tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);
        sliding_window_.pushBackBuffer(WINDOW_SIZE - 1, tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);
    }
    sliding_window_.copyFrame(WINDOW_SIZE - 1, WINDOW_SIZE);
    sliding_window_.createNewPreintegration(WINDOW_SIZE, prev_acc_, prev_gyro_);
    sliding_window_.clearBuffer(WINDOW_SIZE);
    feature_manager_.removeFront(frame_count_);
}

void Estimator::slideWindowOldKeyframe() {
    Matrix3d back_R0 = sliding_window_.front().R;
    Vector3d back_P0 = sliding_window_.front().P;

    double t_0 = sliding_window_.front().timestamp;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sliding_window_.swapBuffer(i, i + 1);
        sliding_window_.swapFrame(i, i + 1);
    }
    sliding_window_.copyFrame(WINDOW_SIZE, WINDOW_SIZE - 1);
    sliding_window_.createNewPreintegration(WINDOW_SIZE, prev_acc_, prev_gyro_);
    sliding_window_.clearBuffer(WINDOW_SIZE);

    cleanupOldImageFrames(t_0);

    bool shift_depth = (solver_flag_ == common::SolverFlag::NON_LINEAR) ? true : false;
    if (shift_depth) {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * r_ic_;
        R1 = sliding_window_.front().R * r_ic_;
        P0 = back_P0 + back_R0 * t_ic_;
        P1 = sliding_window_.front().P + sliding_window_.front().R * t_ic_;
        feature_manager_.removeBackShiftDepth(R0, P0, R1, P1);
    } else
        feature_manager_.removeBack();
}

void Estimator::solveOdometry() {
    if (frame_count_ < WINDOW_SIZE)
        return;

    if (solver_flag_ == common::SolverFlag::NON_LINEAR) {
        feature_manager_.triangulateAcrossAllViews(sliding_window_, t_ic_, r_ic_);

        optimizer_.optimize(marginalization_flag_);

        // Update extrinsic parameters from optimizer
        t_ic_ = optimizer_.getTic();
        r_ic_ = optimizer_.getRic();
    }
}

std::vector<Eigen::Vector3d> Estimator::getSlidingWindowMapPoints() const {
    std::lock_guard<std::mutex> lock(estimator_mutex_);
    std::vector<Eigen::Vector3d> new_points;
    if (solver_flag_ == common::SolverFlag::NON_LINEAR) {
        for (const auto& it_per_id : feature_manager_.feature_bank_) {
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            if (it_per_id.estimated_depth > 0 && it_per_id.solve_flag == 1) {
                Eigen::Vector3d point_normalized = it_per_id.feature_per_frame[0].ray_vector;
                Eigen::Vector3d point_3d = point_normalized * it_per_id.estimated_depth;
                int frame_idx = it_per_id.start_frame;
                if (frame_idx < WINDOW_SIZE) {
                    Eigen::Matrix3d R_wc = sliding_window_[frame_idx].R * r_ic_;
                    Eigen::Vector3d t_wc = sliding_window_[frame_idx].P + sliding_window_[frame_idx].R * t_ic_;
                    Eigen::Vector3d point_world = R_wc * point_3d + t_wc;
                    if (point_world.allFinite()) {
                        new_points.push_back(point_world);
                    }
                }
            }
        }
    }
    return new_points;
}

}  // namespace backend
