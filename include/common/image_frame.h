#ifndef COMMON__IMAGE_FRAME_H
#define COMMON__IMAGE_FRAME_H

#include <eigen3/Eigen/Dense>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "backend/factor/imu_factor.h"

namespace common {

using ImageData = std::map<int, Eigen::Matrix<double, 7, 1>>;

/**
 * @brief ImageFrame class for storing image-related data and pose information
 *
 * This class encapsulates all information related to a single image frame,
 * including feature points, timestamp, pose (R, T), and IMU pre-integration
 * data.
 */
class ImageFrame {
public:
    /**
     * @brief Default constructor
     */
    ImageFrame() : t(0.0), is_key_frame(false) {}

    /**
     * @brief Constructor with feature points and timestamp
     * @param _points Feature points map: feature_id -> vector<camera_id,
     * feature_data>
     * @param _t Timestamp of the image frame
     */
    ImageFrame(const ImageData& _points, double _t)
        : points(_points), t(_t), is_key_frame(false) {
        R = Eigen::Matrix3d::Identity();
        T = Eigen::Vector3d::Zero();
    }

    /**
     * @brief Destructor
     */
    ~ImageFrame() = default;

    /**
     * @brief Copy constructor - deleted due to unique_ptr
     */
    ImageFrame(const ImageFrame& other) = delete;

    /**
     * @brief Assignment operator - deleted due to unique_ptr
     */
    ImageFrame& operator=(const ImageFrame& other) = delete;

    /**
     * @brief Move constructor
     */
    ImageFrame(ImageFrame&& other) noexcept
        : points(std::move(other.points)),
          t(other.t),
          R(std::move(other.R)),
          T(std::move(other.T)),
          pre_integration(std::move(other.pre_integration)),
          is_key_frame(other.is_key_frame) {}

    /**
     * @brief Move assignment operator
     */
    ImageFrame& operator=(ImageFrame&& other) noexcept {
        if (this != &other) {
            points = std::move(other.points);
            t = other.t;
            R = std::move(other.R);
            T = std::move(other.T);
            pre_integration = std::move(other.pre_integration);
            is_key_frame = other.is_key_frame;
        }
        return *this;
    }

    /**
     * @brief Get the number of feature points in this frame
     * @return Total number of feature observations
     */
    size_t getFeatureCount() const {
        size_t count = 0;
        for (const auto& feature : points) {
            count += feature.second.size();
        }
        return count;
    }

    /**
     * @brief Check if this frame has enough features for processing
     * @param min_features Minimum required number of features
     * @return true if frame has enough features
     */
    bool hasEnoughFeatures(size_t min_features = 10) const {
        return getFeatureCount() >= min_features;
    }

    /**
     * @brief Get pose as a 4x4 transformation matrix
     * @return 4x4 transformation matrix [R t; 0 1]
     */
    Eigen::Matrix4d getPoseMatrix() const {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = R;
        pose.block<3, 1>(0, 3) = T;
        return pose;
    }

    /**
     * @brief Set pose from a 4x4 transformation matrix
     * @param pose 4x4 transformation matrix
     */
    void setPoseMatrix(const Eigen::Matrix4d& pose) {
        R = pose.block<3, 3>(0, 0);
        T = pose.block<3, 1>(0, 3);
    }

public:
    /// Feature points: feature_id -> vector<camera_id, feature_measurement>
    /// feature_measurement: [u, v, 1, u_velocity, v_velocity, depth,
    /// depth_covariance]
    ImageData points;

    /// Timestamp of the image frame
    double t;

    /// Rotation matrix (camera to world)
    Eigen::Matrix3d R;

    /// Translation vector (camera position in world frame)
    Eigen::Vector3d T;

    /// IMU pre-integration data from previous frame to this frame
    std::unique_ptr<backend::factor::IntegrationBase> pre_integration;

    /// Flag indicating if this is a keyframe
    bool is_key_frame;
};

}  // namespace common

#endif  // COMMON__IMAGE_FRAME_H