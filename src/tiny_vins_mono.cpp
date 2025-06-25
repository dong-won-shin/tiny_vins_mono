#include <iostream>
#include <thread>

#include "utility/config.h"
#include "backend/estimator.h"
#include "utility/measurement_processor.h"
#include "utility/visualizer.h"

Estimator estimator;
Visualizer visualizer;

void setParameters() {
    estimator.setParameter();
}

void updateCameraPose(double timestamp) {
    if (estimator.solver_flag_ == SolverFlag::NON_LINEAR) {
        int window_size = g_config.estimator.window_size;
        Eigen::Vector3d body_position = estimator.sliding_window_[window_size].P;
        Eigen::Matrix3d body_rotation = estimator.sliding_window_[window_size].R;
        if (!body_position.allFinite() || !body_rotation.allFinite()) {
            std::cerr << "Invalid pose data detected, skipping..." << std::endl;
            return;
        }
        Eigen::Vector3d camera_position = body_position + body_rotation * estimator.t_ic_;
        Eigen::Matrix3d camera_rotation = body_rotation * estimator.r_ic_;
        if (visualizer.isRunning()) {
            visualizer.updateCameraPose(camera_position, camera_rotation, timestamp);
        }
        static size_t last_printed_count = 0;
        static std::vector<Eigen::Vector3d> temp_poses;
        temp_poses.push_back(camera_position);
        if (temp_poses.size() > last_printed_count && temp_poses.size() % 5 == 0) {
            std::cout << "🎯 Frame " << temp_poses.size() 
                        << " | Time: " << std::fixed << std::setprecision(3) << timestamp 
                        << " | Cam Pos: [" << std::fixed << std::setprecision(2)
                        << camera_position.x() << ", " << camera_position.y() << ", " << camera_position.z() << "]" << std::endl;
            last_printed_count = temp_poses.size();
        }
        static size_t last_saved_count = 0;
        if (temp_poses.size() > last_saved_count && temp_poses.size() % 50 == 0) {
            visualizer.saveTrajectoryToFile();
            last_saved_count = temp_poses.size();
        }
    }
}

void updateFeaturePoints3D() {
    std::vector<Eigen::Vector3d> new_points;
    if (estimator.solver_flag_ == SolverFlag::NON_LINEAR) {
        new_points = estimator.getSlidingWindowMapPoints();
    }
    std::vector<Eigen::Vector3d> valid_points;
    for (const auto& pt : new_points) {
        if (pt.allFinite() && !std::isnan(pt.x()) && !std::isnan(pt.y()) && !std::isnan(pt.z())) {
            valid_points.push_back(pt);
        }
    }
    if (visualizer.isRunning()) {
        visualizer.updateFeaturePoints3D(valid_points);
    }
}

void updateVisualization(double timestamp) {
    updateCameraPose(timestamp);
    updateFeaturePoints3D();
}

void process()
{
    MeasurementProcessor processor;
    std::string imu_filepath = g_config.dataset_path + "/mav0/imu0/data.csv";
    std::string image_csv_filepath = g_config.dataset_path + "/mav0/cam0/data.csv";
    std::string image_dir = g_config.dataset_path + "/mav0/cam0/data";
    std::string config_filepath = g_config.config_filepath;

    std::cout << "IMU file: " << imu_filepath << std::endl;
    std::cout << "Image CSV file: " << image_csv_filepath << std::endl;
    std::cout << "Image directory: " << image_dir << std::endl;
    std::cout << "Config file: " << config_filepath << std::endl;
    if (!processor.initialize(imu_filepath, image_csv_filepath, image_dir, config_filepath)) {
        std::cerr << "MeasurementProcessor initialization failed" << std::endl;
        return;
    } else {
        std::cout << "MeasurementProcessor initialization succeeded" << std::endl;
    }

    const auto& image_file_data = processor.getImageFileData();
    double current_time = -1;
    int32_t measurement_id = 0;
    for (const auto& image_file_data_item : image_file_data) {
        std::cout << "\nProcessing file: " << image_file_data_item.filename << std::endl;
        if (measurement_id++ % (g_config.frame_skip + 1) != 0) 
        {
            std::cout << "skip frame " << (measurement_id-1) << std::endl;
            continue;
        }
        MeasurementMsg measurement = processor.createMeasurementMsg(measurement_id, image_file_data_item);
        auto imu_msg = measurement.imu_msg;
        auto image_msg = measurement.image_feature_msg;
        double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
        for (const auto& imu_data : imu_msg) {
            double t = imu_data.timestamp;
            double img_t = image_msg.timestamp;
            if (t <= img_t) {
                if (current_time < 0)
                    current_time = t;
                double dt = t - current_time;
                current_time = t;
                dx = imu_data.linear_acc_x;
                dy = imu_data.linear_acc_y;
                dz = imu_data.linear_acc_z;
                rx = imu_data.angular_vel_x;
                ry = imu_data.angular_vel_y;
                rz = imu_data.angular_vel_z;
                estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
            } 
            else {
                double dt_1 = img_t - current_time;
                double dt_2 = t - img_t;
                current_time = img_t;
                double w1 = dt_2 / (dt_1 + dt_2);
                double w2 = dt_1 / (dt_1 + dt_2);
                dx = w1 * dx + w2 * imu_data.linear_acc_x;
                dy = w1 * dy + w2 * imu_data.linear_acc_y;
                dz = w1 * dz + w2 * imu_data.linear_acc_z;
                rx = w1 * rx + w2 * imu_data.angular_vel_x;
                ry = w1 * ry + w2 * imu_data.angular_vel_y;
                rz = w1 * rz + w2 * imu_data.angular_vel_z;
                estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
            }
        }
        ImageData image_data;
        for (unsigned int i = 0; i < image_msg.points_count; i++) {                    
            int v = image_msg.channel_data[0][i] + 0.5;
            int feature_id = v;
            double x = image_msg.feature_points[i].x;
            double y = image_msg.feature_points[i].y;
            double z = image_msg.feature_points[i].z;
            double p_u = image_msg.channel_data[1][i];
            double p_v = image_msg.channel_data[2][i];
            double velocity_x = image_msg.channel_data[3][i];
            double velocity_y = image_msg.channel_data[4][i];
            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            image_data[feature_id].emplace_back(xyz_uv_velocity);
        }
        if (!image_data.empty()) {
            estimator.processImage(image_data, image_msg.timestamp);
        } else {
            std::cerr << "Warning: Empty image_data for measurement ID " << measurement_id << std::endl;
        }

        updateVisualization(image_msg.timestamp);
    }
    std::cout << "\n🎯 Saving final complete trajectory..." << std::endl;
    visualizer.saveTrajectoryToFile();
}

int main(int argc, char* argv[]) {
     if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <config_file>" << std::endl;
        std::cout << "Example: " << argv[0] << " ./config/config.yaml" << std::endl;
        return 1;
    }

    std::string config_file = argv[1];
    if (!utility::g_config.loadFromYaml(config_file)) {
        std::cout << "Failed to load config from " << config_file << std::endl;
        return 1;
    }
    utility::g_config.print();

    setParameters();

    std::cout << "Starting with Visualizer 3D visualization..." << std::endl;
    if (!visualizer.initialize(config_file)) {
        std::cerr << "Failed to initialize Visualizer" << std::endl;
        return 1;
    }
    std::cout << "Visualizer initialized successfully" << std::endl;

    std::thread process_thread(process);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "Starting Visualizer in main thread..." << std::endl;
    visualizer.pangolinViewerThread();
    
    std::cout << "\n✅ All measurement files processed!" << std::endl;
    if (process_thread.joinable()) {
        process_thread.join();
    }
    std::cout << "Process thread joined" << std::endl;
    return 0;

}