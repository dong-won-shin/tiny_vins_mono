#include "vio_system.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

using namespace Eigen;

VIOSystem::VIOSystem(std::shared_ptr<utility::Config> config) 
    : config_(config) {
    measurement_processor_ = std::make_unique<MeasurementProcessor>();
    vio_estimator_ = std::make_unique<backend::Estimator>();
    visualizer_ = std::make_unique<Visualizer>();
    result_logger_ = std::make_unique<utility::TestResultLogger>();
}

VIOSystem::~VIOSystem() {
    shutdown();
}

bool VIOSystem::initialize() {
    if (!config_) {
        std::cerr << "Error: Configuration not provided" << std::endl;
        return false;
    }
    
    vioInitialize();
    return true;
}

void VIOSystem::processSequence() {
    vio_process_thread_ = std::make_unique<std::thread>([this]() {
        this->vioProcess();
    });

    // Wait for visualizer to be initialized
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Starting Visualizer in main thread..." << std::endl;
    visualizer_->pangolinViewerThread();

    std::cout << "\n✅ All measurement files processed!" << std::endl;
    if (vio_process_thread_ && vio_process_thread_->joinable()) {
        vio_process_thread_->join();
    }

    std::cout << "Process thread joined" << std::endl;
}

void VIOSystem::shutdown() {
    if (vio_process_thread_ && vio_process_thread_->joinable()) {
        vio_process_thread_->join();
    }
}

void VIOSystem::vioInitialize() {
    const std::string imu_filepath = config_->dataset_path + "/mav0/imu0/data.csv";
    const std::string image_csv_filepath = config_->dataset_path + "/mav0/cam0/data.csv";
    const std::string image_dirpath = config_->dataset_path + "/mav0/cam0/data";
    const std::string config_filepath = config_->config_filepath;

    std::cout << "IMU file: " << imu_filepath << std::endl;
    std::cout << "Image CSV file: " << image_csv_filepath << std::endl;
    std::cout << "Image directory: " << image_dirpath << std::endl;
    std::cout << "Config file: " << config_filepath << std::endl;

    measurement_processor_->initialize(imu_filepath, image_csv_filepath, image_dirpath, config_filepath);
    vio_estimator_->setParameter();
    visualizer_->initialize();
    result_logger_->initialize(config_filepath);
}

void VIOSystem::updateCameraPose(double timestamp) {
    if (vio_estimator_->solver_flag_ == common::SolverFlag::NON_LINEAR) {
        int window_size = config_->estimator.window_size;
        Eigen::Vector3d body_position = vio_estimator_->sliding_window_[window_size].P;
        Eigen::Matrix3d body_rotation = vio_estimator_->sliding_window_[window_size].R;
        if (!body_position.allFinite() || !body_rotation.allFinite()) {
            std::cerr << "Invalid pose data detected, skipping..." << std::endl;
            return;
        }
        Eigen::Vector3d camera_position = body_position + body_rotation * vio_estimator_->t_ic_;
        Eigen::Matrix3d camera_rotation = body_rotation * vio_estimator_->r_ic_;

        if (visualizer_->isRunning()) {
            visualizer_->updateCameraPose(camera_position, camera_rotation, timestamp);
        }

        result_logger_->addPose(camera_position, camera_rotation, timestamp);

        static size_t last_printed_count = 0;
        static std::vector<Eigen::Vector3d> temp_poses;
        temp_poses.push_back(camera_position);
        if (temp_poses.size() > last_printed_count && temp_poses.size() % 5 == 0) {
            std::cout << "🎯 Frame " << temp_poses.size() << " | Time: " << std::fixed << std::setprecision(3)
                      << timestamp << " | Cam Pos: [" << std::fixed << std::setprecision(2) << camera_position.x()
                      << ", " << camera_position.y() << ", " << camera_position.z() << "]" << std::endl;
            last_printed_count = temp_poses.size();
        }
        static size_t last_saved_count = 0;
        if (temp_poses.size() > last_saved_count && temp_poses.size() % 50 == 0) {
            result_logger_->saveTrajectoryToFile();
            last_saved_count = temp_poses.size();
        }
    }
}

void VIOSystem::updateFeaturePoints3D() {
    std::vector<Eigen::Vector3d> new_points;
    if (vio_estimator_->solver_flag_ == common::SolverFlag::NON_LINEAR) {
        new_points = vio_estimator_->getSlidingWindowMapPoints();
    }
    std::vector<Eigen::Vector3d> valid_points;
    for (const auto& pt : new_points) {
        if (pt.allFinite() && !std::isnan(pt.x()) && !std::isnan(pt.y()) && !std::isnan(pt.z())) {
            valid_points.push_back(pt);
        }
    }
    if (visualizer_->isRunning()) {
        visualizer_->updateFeaturePoints3D(valid_points);
    }
}

void VIOSystem::updateVisualization(double timestamp) {
    updateCameraPose(timestamp);
    updateFeaturePoints3D();
}

void VIOSystem::processIMUData(const std::vector<utility::IMUMsg>& imu_msg, const utility::ImageFeatureMsg& image_msg, double& current_time) {
    for (const auto& imu_data : imu_msg) {
        double t = imu_data.timestamp;
        double img_t = image_msg.timestamp;
        double linear_acc_x = 0, linear_acc_y = 0, linear_acc_z = 0, angular_vel_x = 0, angular_vel_y = 0,
               angular_vel_z = 0;
        if (t <= img_t) {
            if (current_time < 0)
                current_time = t;
            double dt = t - current_time;
            current_time = t;
            linear_acc_x = imu_data.linear_acc_x;
            linear_acc_y = imu_data.linear_acc_y;
            linear_acc_z = imu_data.linear_acc_z;
            angular_vel_x = imu_data.angular_vel_x;
            angular_vel_y = imu_data.angular_vel_y;
            angular_vel_z = imu_data.angular_vel_z;
            vio_estimator_->processIMU(dt, Vector3d(linear_acc_x, linear_acc_y, linear_acc_z),
                                     Vector3d(angular_vel_x, angular_vel_y, angular_vel_z));
        } else {
            double dt_1 = img_t - current_time;
            double dt_2 = t - img_t;
            current_time = img_t;
            double w1 = dt_2 / (dt_1 + dt_2);
            double w2 = dt_1 / (dt_1 + dt_2);
            linear_acc_x = w1 * linear_acc_x + w2 * imu_data.linear_acc_x;
            linear_acc_y = w1 * linear_acc_y + w2 * imu_data.linear_acc_y;
            linear_acc_z = w1 * linear_acc_z + w2 * imu_data.linear_acc_z;
            angular_vel_x = w1 * angular_vel_x + w2 * imu_data.angular_vel_x;
            angular_vel_y = w1 * angular_vel_y + w2 * imu_data.angular_vel_y;
            angular_vel_z = w1 * angular_vel_z + w2 * imu_data.angular_vel_z;
            vio_estimator_->processIMU(dt_1, Vector3d(linear_acc_x, linear_acc_y, linear_acc_z),
                                     Vector3d(angular_vel_x, angular_vel_y, angular_vel_z));
        }
    }
}

void VIOSystem::processImageData(const utility::ImageFeatureMsg& image_msg) {
    common::ImageData image_data;
    for (unsigned int i = 0; i < image_msg.points_count; i++) {
        int feature_id = image_msg.channel_data[0][i];
        double r_x = image_msg.ray_vectors[i].x;
        double r_y = image_msg.ray_vectors[i].y;
        double r_z = image_msg.ray_vectors[i].z;
        double p_u = image_msg.channel_data[1][i];
        double p_v = image_msg.channel_data[2][i];
        double v_x = image_msg.channel_data[3][i];
        double v_y = image_msg.channel_data[4][i];
        Eigen::Matrix<double, 7, 1> ray_obs_vel;
        ray_obs_vel << r_x, r_y, r_z, p_u, p_v, v_x, v_y;
        image_data[feature_id] = ray_obs_vel;
    }
    if (!image_data.empty()) {
        vio_estimator_->processImage(image_data, image_msg.timestamp);
    } else {
        std::cerr << "Warning: Empty image_data" << std::endl;
    }
}

void VIOSystem::processSingleFrame(const utility::MeasurementMsg& measurement, double& current_time, int32_t measurement_id) {
    auto imu_msg = measurement.imu_msg;
    auto image_msg = measurement.image_feature_msg;

    // Process IMU data
    processIMUData(imu_msg, image_msg, current_time);

    // Process image data
    processImageData(image_msg);

    updateVisualization(image_msg.timestamp);
}

void VIOSystem::vioProcess() {
    const auto& image_file_data = measurement_processor_->getImageFileData();
    double current_time = -1;
    int32_t measurement_id = 0;
    for (const auto& image_file_data_item : image_file_data) {
        std::cout << "\nProcessing file: " << image_file_data_item.filename << std::endl;
        if (measurement_id++ % (config_->frame_skip + 1) != 0) {
            std::cout << "skip frame " << (measurement_id - 1) << std::endl;
            continue;
        }

        MeasurementMsg measurement = measurement_processor_->createMeasurementMsg(measurement_id, image_file_data_item);
        processSingleFrame(measurement, current_time, measurement_id);
    }

    std::cout << "\n🎯 Saving final complete trajectory..." << std::endl;
    result_logger_->saveTrajectoryToFile();
}