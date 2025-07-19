#include "utility/config.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

namespace utility {
// Global configuration instance
Config g_config;

bool Config::loadFromYaml(const std::string& yaml_path) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);

        std::cout << "Loading configuration from: " << yaml_path << std::endl;

        // Load camera configuration
        if (config["image_width"])
            camera.col = config["image_width"].as<double>();
        if (config["image_height"])
            camera.row = config["image_height"].as<double>();

        // Load projection parameters
        if (config["projection_parameters"]) {
            auto proj = config["projection_parameters"];
            if (proj["fx"])
                camera.fx = proj["fx"].as<double>();
            if (proj["fy"])
                camera.fy = proj["fy"].as<double>();
            if (proj["cx"])
                camera.cx = proj["cx"].as<double>();
            if (proj["cy"])
                camera.cy = proj["cy"].as<double>();

            // Set focal_length as average of fx and fy
            camera.focal_length = (camera.fx + camera.fy) / 2.0;
        }

        // Load extrinsic parameters (OpenCV matrix format)
        if (config["extrinsicRotation"]) {
            auto ext_rot = config["extrinsicRotation"];
            if (ext_rot["data"]) {
                auto data = ext_rot["data"];
                if (data.size() >= 9) {
                    camera.r_ic << data[0].as<double>(), data[1].as<double>(), data[2].as<double>(),
                        data[3].as<double>(), data[4].as<double>(), data[5].as<double>(), data[6].as<double>(),
                        data[7].as<double>(), data[8].as<double>();
                }
            }
        }

        if (config["extrinsicTranslation"]) {
            auto ext_trans = config["extrinsicTranslation"];
            if (ext_trans["data"]) {
                auto data = ext_trans["data"];
                if (data.size() >= 3) {
                    camera.t_ic << data[0].as<double>(), data[1].as<double>(), data[2].as<double>();
                }
            }
        }

        // Load feature tracker configuration (top level parameters)
        if (config["max_cnt"])
            feature_tracker.max_cnt = config["max_cnt"].as<int>();
        if (config["min_dist"])
            feature_tracker.min_dist = config["min_dist"].as<int>();
        if (config["F_threshold"])
            feature_tracker.f_threshold = config["F_threshold"].as<double>();
        if (config["show_track"])
            feature_tracker.show_track = config["show_track"].as<int>();
        if (config["equalize"])
            feature_tracker.equalize = config["equalize"].as<int>();
        if (config["fisheye"])
            feature_tracker.fisheye = config["fisheye"].as<int>();

        // Load estimator configuration (top level parameters)
        if (config["max_solver_time"])
            estimator.solver_time = config["max_solver_time"].as<double>();
        if (config["max_num_iterations"])
            estimator.num_iterations = config["max_num_iterations"].as<int>();
        if (config["keyframe_parallax"])
            estimator.min_parallax = config["keyframe_parallax"].as<double>();
        if (config["acc_n"])
            estimator.acc_n = config["acc_n"].as<double>();
        if (config["acc_w"])
            estimator.acc_w = config["acc_w"].as<double>();
        if (config["gyr_n"])
            estimator.gyr_n = config["gyr_n"].as<double>();
        if (config["gyr_w"])
            estimator.gyr_w = config["gyr_w"].as<double>();
        if (config["g_norm"])
            estimator.g = Eigen::Vector3d(0.0, 0.0, config["g_norm"].as<double>());

        // Load processing parameters
        if (config["frame_skip"])
            frame_skip = config["frame_skip"].as<int>();
        if (config["start_frame"])
            start_frame = config["start_frame"].as<int>();
        if (config["end_frame"])
            end_frame = config["end_frame"].as<int>();
        if (config["dataset_path"])
            dataset_path = config["dataset_path"].as<std::string>();
        config_filepath = yaml_path;

        // Set default values for missing parameters
        if (estimator.init_depth == 0.0)
            estimator.init_depth = 5.0;
        if (estimator.num_of_features == 0)
            estimator.num_of_features = 1000;
        if (feature_tracker.window_size == 0)
            feature_tracker.window_size = 20;

        std::cout << "Configuration loaded successfully from: " << yaml_path << std::endl;
        return true;

    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading configuration from " << yaml_path << ": " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error loading configuration from " << yaml_path << ": " << e.what() << std::endl;
        return false;
    }
}

void Config::print() const {
    std::cout << "=== Configuration ===" << std::endl;

    std::cout << "Processing:" << std::endl;
    std::cout << "  frame_skip: " << frame_skip << std::endl;
    std::cout << "  start_frame: " << start_frame << std::endl;
    std::cout << "  end_frame: " << end_frame << std::endl;

    std::cout << "Camera:" << std::endl;
    std::cout << "  focal_length: " << camera.focal_length << std::endl;
    std::cout << "  row: " << camera.row << std::endl;
    std::cout << "  col: " << camera.col << std::endl;
    std::cout << "  fx: " << camera.fx << std::endl;
    std::cout << "  fy: " << camera.fy << std::endl;
    std::cout << "  cx: " << camera.cx << std::endl;
    std::cout << "  cy: " << camera.cy << std::endl;
    std::cout << "  t_ic: " << camera.t_ic.transpose() << std::endl;

    std::cout << "Feature Tracker:" << std::endl;
    std::cout << "  max_cnt: " << feature_tracker.max_cnt << std::endl;
    std::cout << "  min_dist: " << feature_tracker.min_dist << std::endl;
    std::cout << "  window_size: " << feature_tracker.window_size << std::endl;
    std::cout << "  f_threshold: " << feature_tracker.f_threshold << std::endl;
    std::cout << "  show_track: " << feature_tracker.show_track << std::endl;
    std::cout << "  equalize: " << feature_tracker.equalize << std::endl;
    std::cout << "  fisheye: " << feature_tracker.fisheye << std::endl;

    std::cout << "Estimator:" << std::endl;
    std::cout << "  window_size: " << estimator.window_size << std::endl;
    std::cout << "  num_iterations: " << estimator.num_iterations << std::endl;
    std::cout << "  solver_time: " << estimator.solver_time << std::endl;
    std::cout << "  min_parallax: " << estimator.min_parallax << std::endl;
    std::cout << "  init_depth: " << estimator.init_depth << std::endl;
    std::cout << "  acc_n: " << estimator.acc_n << std::endl;
    std::cout << "  acc_w: " << estimator.acc_w << std::endl;
    std::cout << "  gyr_n: " << estimator.gyr_n << std::endl;
    std::cout << "  gyr_w: " << estimator.gyr_w << std::endl;
    std::cout << "  gravity: " << estimator.g.transpose() << std::endl;
    std::cout << "  num_of_features: " << estimator.num_of_features << std::endl;

    std::cout << "===================" << std::endl;
}

}  // namespace utility