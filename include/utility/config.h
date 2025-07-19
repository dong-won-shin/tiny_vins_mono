#ifndef UTILITY__CONFIG_H
#define UTILITY__CONFIG_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace utility {

const int WINDOW_SIZE = 10;
const int NUM_OF_FEATURES = 1000;

enum SizeParameterization { SIZE_POSE = 7, SIZE_SPEEDANDBIAS = 9, SIZE_FEATURE = 1 };
enum StateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };

// Camera configuration
struct CameraConfig {
    double focal_length = 460.0;
    double row = 480.0;
    double col = 752.0;
    Eigen::Matrix3d r_ic = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_ic = Eigen::Vector3d::Zero();

    // Camera intrinsic parameters
    double fx = 460.0;
    double fy = 460.0;
    double cx = 376.0;  // col * 0.5
    double cy = 240.0;  // row * 0.5
};

// Feature tracker configuration
struct FeatureTrackerConfig {
    int max_cnt = 150;
    int min_dist = 30;
    int window_size = 20;
    double f_threshold = 1.0;
    int show_track = 1;
    int equalize = 1;
    int fisheye = 0;

    // Image processing
    std::string fisheye_mask;
};

// Estimator configuration
struct EstimatorConfig {
    int window_size = 10;
    int num_iterations = 10;
    double solver_time = 0.05;
    double min_parallax = 10.0;
    double init_depth = 5.0;

    // IMU noise parameters
    double acc_n = 0.08;
    double acc_w = 0.00004;
    double gyr_n = 0.004;
    double gyr_w = 2.0e-6;

    // Gravity vector
    Eigen::Vector3d g = Eigen::Vector3d(0.0, 0.0, 9.81007);

    // Feature parameters
    int num_of_features = NUM_OF_FEATURES;
};

// Main configuration structure
struct Config {
    CameraConfig camera;
    FeatureTrackerConfig feature_tracker;
    EstimatorConfig estimator;

    // Processing parameters
    int frame_skip = 2;  // Skip frames for processing
    int start_frame = 0;  // Start processing from this frame (0-based index)
    int end_frame = -1;   // End processing at this frame (-1 = process all frames)
    std::string dataset_path = "";
    std::string config_filepath = "";

    // Load configuration from YAML file
    bool loadFromYaml(const std::string& yaml_path);

    // Print configuration for debugging
    void print() const;
};

// Global configuration instance
extern Config g_config;

}  // namespace utility

#endif  // UTILITY__CONFIG_H