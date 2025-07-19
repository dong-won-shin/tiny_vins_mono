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
    imu_graph_visualizer_ = std::make_unique<utility::IMUGraphVisualizer>();
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
    if (imu_graph_visualizer_) {
        imu_graph_visualizer_->stop();
    }
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
    imu_graph_visualizer_->initialize(1000, 720, 300);
    imu_graph_visualizer_->start();
    result_logger_->initialize(config_filepath);
}


void VIOSystem::onFrameProcessed(const utility::MeasurementMsg& measurement, double& current_time, int32_t measurement_id) {
    auto imu_msg = measurement.imu_msg;
    auto image_msg = measurement.image_feature_msg;

    // Process IMU data
    processIMUData(imu_msg, image_msg, current_time);

    // Process image data
    processImageData(image_msg);

    // Update visualization
    updateVisualization(image_msg.timestamp);
}

void VIOSystem::onSequenceComplete() {
    std::cout << "\n🎯 Saving final complete trajectory..." << std::endl;
    result_logger_->saveTrajectoryToFile();
}

void VIOSystem::vioProcess() {
    const auto& image_file_data = measurement_processor_->getImageFileData();
    double current_time = -1;
    int32_t measurement_id = 0;
    
    // Get frame range parameters
    int start_frame = config_->start_frame;
    int end_frame = config_->end_frame;
    int total_frames = static_cast<int>(image_file_data.size());
    
    // Validate frame range parameters
    if (start_frame < 0) {
        start_frame = 0;
        std::cout << "Warning: start_frame < 0, setting to 0" << std::endl;
    }
    if (end_frame < 0 || end_frame >= total_frames) {
        end_frame = total_frames - 1;
        std::cout << "Info: end_frame set to " << end_frame << " (total frames: " << total_frames << ")" << std::endl;
    }
    if (start_frame > end_frame) {
        std::cerr << "Error: start_frame (" << start_frame << ") > end_frame (" << end_frame << ")" << std::endl;
        return;
    }
    
    std::cout << "Processing frames " << start_frame << " to " << end_frame << " (total: " << (end_frame - start_frame + 1) << " frames)" << std::endl;
    
    for (int frame_idx = 0; frame_idx < total_frames; ++frame_idx) {
        // Skip frames outside the specified range
        if (frame_idx < start_frame || frame_idx > end_frame) {
            continue;
        }
        
        const auto& image_file_data_item = image_file_data[frame_idx];
        std::cout << "\nProcessing file: " << image_file_data_item.filename << " (frame " << frame_idx << "/" << total_frames - 1 << ")" << std::endl;
        
        if (measurement_id++ % (config_->frame_skip + 1) != 0) {
            std::cout << "skip frame " << (measurement_id - 1) << std::endl;
            continue;
        }

        utility::MeasurementMsg measurement = measurement_processor_->createMeasurementMsg(measurement_id, image_file_data_item);
        onFrameProcessed(measurement, current_time, measurement_id);
    }

    onSequenceComplete();
}

void VIOSystem::processIMUData(const std::vector<utility::IMUMsg>& imu_msg, const utility::ImageFeatureMsg& image_msg, double& current_time) {
    Vector3d prev_acc = Vector3d::Zero();
    Vector3d prev_gyro = Vector3d::Zero();
    Vector3d curr_acc, curr_gyro;

    for (const auto& imu_data : imu_msg) {
        const double imu_time = imu_data.timestamp;
        const double image_time = image_msg.timestamp;

        if (imu_time <= image_time) {
            if (current_time < 0.0) {
                current_time = imu_time;
            }
            const double dt = imu_time - current_time;
            current_time = imu_time;
            
            curr_acc = extractAcceleration(imu_data);
            curr_gyro = extractAngularVelocity(imu_data);

            // Update IMU graph visualization
            if (imu_graph_visualizer_ && imu_graph_visualizer_->isRunning()) {
                imu_graph_visualizer_->addIMUData(imu_time, curr_acc, curr_gyro);
            }
            
            vio_estimator_->processIMU(dt, curr_acc, curr_gyro);
        } else {
            const double dt_to_image = image_time - current_time;
            current_time = image_time;
            
            interpolateIMUData(prev_acc, prev_gyro, imu_data, dt_to_image, imu_time - image_time, curr_acc, curr_gyro);

            // Update IMU graph visualization with interpolated data
            if (imu_graph_visualizer_ && imu_graph_visualizer_->isRunning()) {
                imu_graph_visualizer_->addIMUData(image_time, curr_acc, curr_gyro);
            }
            
            vio_estimator_->processIMU(dt_to_image, curr_acc, curr_gyro);
        }

        prev_acc = curr_acc;
        prev_gyro = curr_gyro;
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

void VIOSystem::updateVisualization(double timestamp) {
    updateCameraPose(timestamp);
    updateFeaturePoints3D();
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
        
        // Camera pose (body + camera offset)
        Eigen::Vector3d camera_position = body_position + body_rotation * vio_estimator_->t_ic_;
        Eigen::Matrix3d camera_rotation = body_rotation * vio_estimator_->r_ic_;

        // IMU pose (body frame - IMU is typically at body origin)
        Eigen::Vector3d imu_position = body_position;
        Eigen::Matrix3d imu_rotation = body_rotation;

        if (visualizer_->isRunning()) {
            visualizer_->updateCameraPose(camera_position, camera_rotation, timestamp);
            visualizer_->updateIMUPose(imu_position, imu_rotation, timestamp);
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

Vector3d VIOSystem::extractAcceleration(const utility::IMUMsg& imu_data) {
    return Vector3d(imu_data.linear_acc_x, imu_data.linear_acc_y, imu_data.linear_acc_z);
}

Vector3d VIOSystem::extractAngularVelocity(const utility::IMUMsg& imu_data) {
    return Vector3d(imu_data.angular_vel_x, imu_data.angular_vel_y, imu_data.angular_vel_z);
}

void VIOSystem::interpolateIMUData(const Vector3d& prev_acc, const Vector3d& prev_gyro, 
                                   const utility::IMUMsg& current_imu, 
                                   double dt1, double dt2, 
                                   Vector3d& interp_acc, Vector3d& interp_gyro) {
    const double total_dt = dt1 + dt2;
    const double w1 = dt2 / total_dt;
    const double w2 = dt1 / total_dt;
    
    const Vector3d current_acc = extractAcceleration(current_imu);
    const Vector3d current_gyro = extractAngularVelocity(current_imu);
    
    interp_acc = w1 * prev_acc + w2 * current_acc;
    interp_gyro = w1 * prev_gyro + w2 * current_gyro;
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