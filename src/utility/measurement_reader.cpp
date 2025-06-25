#include "utility/measurement_reader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

bool readMeasurementFile(const std::string& filepath, MeasurementMsg& measurement) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "파일을 열 수 없습니다: " << filepath << std::endl;
        return false;
    }

    std::string line;
    bool reading_imu = false;
    bool reading_features = false;
    bool reading_channels = false;
    
    // 데이터 초기화
    measurement.imu_msg.clear();
    measurement.image_feature_msg.feature_points.clear();

    while (std::getline(file, line)) {
        // 빈 줄이나 주석 건너뛰기
        if (line.empty() || line[0] == '#') {
            // 측정 ID 파싱
            if (line.find("# Measurement ") != std::string::npos) {
                std::istringstream iss(line);
                std::string hash, measurement_str;
                int id;
                iss >> hash >> measurement_str >> id;
                measurement.measurement_id = id;
            }
            // IMU 데이터 섹션 시작
            else if (line.find("## IMU DATA") != std::string::npos) {
                reading_imu = true;
                reading_features = false;
                reading_channels = false;
            }
            // 이미지 특징점 데이터 섹션 시작
            else if (line.find("## IMAGE FEATURE DATA") != std::string::npos) {
                reading_imu = false;
                reading_features = false;
                reading_channels = false;
            }
            // 타임스탬프 파싱
            else if (line.find("# timestamp: ") != std::string::npos) {
                std::string temp = line.substr(line.find(": ") + 2);
                measurement.image_feature_msg.timestamp = std::stod(temp);
            }
            // 프레임 ID 파싱
            else if (line.find("# frame_id: ") != std::string::npos) {
                measurement.image_feature_msg.frame_id = line.substr(line.find(": ") + 2);
            }
            // 포인트 개수 파싱
            else if (line.find("# points_count: ") != std::string::npos) {
                std::string temp = line.substr(line.find(": ") + 2);
                measurement.image_feature_msg.points_count = std::stoi(temp);
            }
            // 채널 개수 파싱
            else if (line.find("# channels_count: ") != std::string::npos) {
                std::string temp = line.substr(line.find(": ") + 2);
                measurement.image_feature_msg.channels_count = std::stoi(temp);
            }
            // 특징점 섹션 시작
            else if (line.find("### FEATURE POINTS") != std::string::npos) {
                reading_features = true;
                reading_imu = false;
                reading_channels = false;
            }
            // 채널 섹션 시작
            else if (line.find("### CHANNEL") != std::string::npos) {
                reading_channels = true;
                reading_features = false;
                reading_imu = false;
            }
            continue;
        }

        // IMU 데이터 파싱
        if (reading_imu) {
            std::istringstream iss(line);
            std::string token;
            IMUMsg imu_msg;
            
            // 쉼표로 구분된 데이터 파싱
            if (std::getline(iss, token, ',')) {
                imu_msg.timestamp = std::stod(token);
                if (std::getline(iss, token, ',')) imu_msg.linear_acc_x = std::stod(token);
                if (std::getline(iss, token, ',')) imu_msg.linear_acc_y = std::stod(token);
                if (std::getline(iss, token, ',')) imu_msg.linear_acc_z = std::stod(token);
                if (std::getline(iss, token, ',')) imu_msg.angular_vel_x = std::stod(token);
                if (std::getline(iss, token, ',')) imu_msg.angular_vel_y = std::stod(token);
                if (std::getline(iss, token)) imu_msg.angular_vel_z = std::stod(token);
                
                measurement.imu_msg.push_back(imu_msg);
            }
        }
        // 특징점 데이터 파싱
        else if (reading_features) {
            std::istringstream iss(line);
            std::string token;
            Point3D point;
            
            // 쉼표로 구분된 데이터 파싱
            if (std::getline(iss, token, ',')) {
                point.index = std::stoi(token);
                if (std::getline(iss, token, ',')) point.x = std::stod(token);
                if (std::getline(iss, token, ',')) point.y = std::stod(token);
                if (std::getline(iss, token)) point.z = std::stod(token);
                
                measurement.image_feature_msg.feature_points.push_back(point);
            }
        }
        // 채널 데이터 파싱
        else if (reading_channels) {
            std::istringstream iss(line);
            std::string token;
            
            // 쉼표로 구분된 데이터 파싱
            if (std::getline(iss, token, ',')) {
                int32_t index = std::stoi(token);
                if (std::getline(iss, token, ',')) {
                    double value = std::stod(token);
                    measurement.image_feature_msg.channel_data[0].push_back(value);
                }
                if (std::getline(iss, token, ',')) {
                    double value = std::stod(token);
                    measurement.image_feature_msg.channel_data[1].push_back(value);
                }
                if (std::getline(iss, token, ',')) {
                    double value = std::stod(token);
                    measurement.image_feature_msg.channel_data[2].push_back(value);
                }
                if (std::getline(iss, token, ',')) {
                    double value = std::stod(token);
                    measurement.image_feature_msg.channel_data[3].push_back(value);
                }
                if (std::getline(iss, token, ',')) {
                    double value = std::stod(token);
                    measurement.image_feature_msg.channel_data[4].push_back(value);
                }
            }
        }
    }

    file.close();
    return true;
}

void printMeasurementMsg(const MeasurementMsg& measurement) {
    std::cout << "=== Measurement " << measurement.measurement_id << " ===" << std::endl;
    
    // IMU 데이터 출력
    std::cout << "IMU Msg (" << measurement.imu_msg.size() << " samples):" << std::endl;
    for (size_t i = 0; i < std::min(measurement.imu_msg.size(), size_t(5)); ++i) {
        const auto& imu = measurement.imu_msg[i];
        std::cout << "  [" << i << "] Time: " << imu.timestamp 
                  << ", Acc: (" << imu.linear_acc_x << ", " << imu.linear_acc_y << ", " << imu.linear_acc_z << ")"
                  << ", Gyro: (" << imu.angular_vel_x << ", " << imu.angular_vel_y << ", " << imu.angular_vel_z << ")" << std::endl;
    }
    if (measurement.imu_msg.size() > 5) {
        std::cout << "  ... and " << (measurement.imu_msg.size() - 5) << " more samples" << std::endl;
    }
    
    // 이미지 특징점 데이터 출력
    std::cout << "Image Feature Msg:" << std::endl;
    std::cout << "  Timestamp: " << measurement.image_feature_msg.timestamp << std::endl;
    std::cout << "  Frame ID: " << measurement.image_feature_msg.frame_id << std::endl;
    std::cout << "  Points Count: " << measurement.image_feature_msg.points_count << std::endl;
    std::cout << "  Channels Count: " << measurement.image_feature_msg.channels_count << std::endl;
    
    std::cout << "  Feature Points (" << measurement.image_feature_msg.feature_points.size() << " points):" << std::endl;
    for (size_t i = 0; i < std::min(measurement.image_feature_msg.feature_points.size(), size_t(5)); ++i) {
        const auto& point = measurement.image_feature_msg.feature_points[i];
        std::cout << "    [" << point.index << "] (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }
    if (measurement.image_feature_msg.feature_points.size() > 5) {
        std::cout << "    ... and " << (measurement.image_feature_msg.feature_points.size() - 5) << " more points" << std::endl;
    }
    
} 