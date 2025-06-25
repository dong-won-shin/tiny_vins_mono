#include "utility/measurement_processor.h"
#include "utility/config.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <set>
#include <opencv2/opencv.hpp>

MeasurementProcessor::MeasurementProcessor() : prev_image_timestamp_(-1.0) 
{
}

MeasurementProcessor::~MeasurementProcessor() {
}

bool MeasurementProcessor::initialize(const std::string& imu_filepath, 
                                     const std::string& image_csv_filepath,
                                     const std::string& image_dir,
                                     const std::string& config_filepath) {
    std::cout << "=== MeasurementProcessor 초기화 ===" << std::endl;
    
    // Load configuration
    if (!g_config.loadFromYaml(config_filepath)) {
        std::cout << "Failed to load config from " << config_filepath << std::endl;
        return false;
    }
    
    // Print configuration for debugging
    g_config.print();
    
    // Feature tracker 초기화
    feature_tracker_.reset(new feature_tracker::FeatureTracker());
    feature_tracker_->readIntrinsicParameter(config_filepath);
    
    // 데이터 로드
    if (!loadImuData(imu_filepath)) {
        std::cerr << "IMU 데이터 로드 실패" << std::endl;
        return false;
    }
    
    if (!loadImageFileData(image_csv_filepath, image_dir)) {
        std::cerr << "이미지 데이터 로드 실패" << std::endl;
        return false;
    }
    
    // 데이터 범위 출력
    printDataRange();
    
    return true;
}

bool MeasurementProcessor::loadImuData(const std::string& filepath) {
    std::cout << "\n1. IMU 데이터 로드 중..." << std::endl;
    
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "IMU 파일을 열 수 없습니다: " << filepath << std::endl;
        return false;
    }
    
    std::string line;
    // 헤더 건너뛰기
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string token;
        IMUData data;
        
        // timestamp [ns]를 초 단위로 변환
        if (std::getline(iss, token, ',')) {
            data.timestamp = std::stod(token) / 1e9; // ns를 초로 변환
        }
        
        // angular velocity (rad/s)
        if (std::getline(iss, token, ',')) data.angular_vel_x = std::stod(token);
        if (std::getline(iss, token, ',')) data.angular_vel_y = std::stod(token);
        if (std::getline(iss, token, ',')) data.angular_vel_z = std::stod(token);
        
        // linear acceleration (m/s^2)
        if (std::getline(iss, token, ',')) data.linear_acc_x = std::stod(token);
        if (std::getline(iss, token, ',')) data.linear_acc_y = std::stod(token);
        if (std::getline(iss, token)) data.linear_acc_z = std::stod(token);
        
        imu_data_.push_back(data);
    }
    
    file.close();
    std::cout << "IMU 데이터 " << imu_data_.size() << "개 로드 완료" << std::endl;
    return true;
}

bool MeasurementProcessor::loadImageFileData(const std::string& csv_filepath, const std::string& image_dir) {
    std::cout << "\n2. 이미지 데이터 로드 중..." << std::endl;
    
    std::ifstream file(csv_filepath);
    if (!file.is_open()) {
        std::cerr << "이미지 CSV 파일을 열 수 없습니다: " << csv_filepath << std::endl;
        return false;
    }
    
    std::string line;
    // 헤더 건너뛰기
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string token;
        ImageFileData data;
        
        // timestamp [ns]를 초 단위로 변환
        if (std::getline(iss, token, ',')) {
            data.timestamp = std::stod(token) / 1e9; // ns를 초로 변환
        }
        
        // filename
        if (std::getline(iss, token)) {
            data.filename = cleanFilename(token);
            data.full_path = image_dir + "/" + data.filename;
        }
        
        image_file_data_.push_back(data);
    }
    
    file.close();
    std::cout << "이미지 데이터 " << image_file_data_.size() << "개 로드 완료" << std::endl;
    return true;
}

std::string MeasurementProcessor::cleanFilename(const std::string& filename) {
    std::string cleaned = filename;
    // 앞뒤 공백 및 개행 제거
    cleaned.erase(cleaned.find_last_not_of(" \n\r\t") + 1);
    cleaned.erase(0, cleaned.find_first_not_of(" \n\r\t"));
    return cleaned;
}

ImageFeatureMsg MeasurementProcessor::extractImageFeatures(const ImageFileData& image_data) {
    ImageFeatureMsg image_feature_msg;
    
    // 항상 timestamp와 frame_id를 설정 (PUB_THIS_FRAME과 관계없이)
    image_feature_msg.timestamp = image_data.timestamp;
    image_feature_msg.frame_id = image_data.filename;
    
    cv::Mat show_img = cv::imread(image_data.full_path, cv::IMREAD_GRAYSCALE);
    if (show_img.empty()) {
        std::cerr << "이미지를 로드할 수 없습니다: " << image_data.full_path << std::endl;
        return image_feature_msg;
    }
    
    feature_tracker_->readImage(show_img.rowRange(0, g_config.camera.row), image_data.timestamp);

    for (unsigned int i = 0;; i++) {
        bool completed = false;
        completed |= feature_tracker_->updateID(i);
        if (!completed)
            break;
    }

    std::vector<int32_t> id_of_point;
    std::vector<float> u_of_point;
    std::vector<float> v_of_point;
    std::vector<float> velocity_x_of_point;
    std::vector<float> velocity_y_of_point;

    std::set<int> hash_ids;
    auto &un_pts = feature_tracker_->cur_un_pts;
    auto &cur_pts = feature_tracker_->cur_pts;
    auto &ids = feature_tracker_->ids;
    auto &pts_velocity = feature_tracker_->pts_velocity;
    
    for (unsigned int j = 0; j < ids.size(); j++) {
        if (feature_tracker_->track_cnt[j] > 1) {
            int p_id = ids[j];
            hash_ids.insert(p_id);
            image_feature_msg.feature_points.push_back(Point3D{p_id, un_pts[j].x, un_pts[j].y, 1.0});
            id_of_point.push_back(p_id);
            u_of_point.push_back(cur_pts[j].x);
            v_of_point.push_back(cur_pts[j].y);
            velocity_x_of_point.push_back(pts_velocity[j].x);
            velocity_y_of_point.push_back(pts_velocity[j].y);
        }
    }
    
    image_feature_msg.points_count = image_feature_msg.feature_points.size();
    image_feature_msg.channels_count = 5;
    image_feature_msg.channel_data[0] = ChannelData(id_of_point.begin(), id_of_point.end());
    image_feature_msg.channel_data[1] = ChannelData(u_of_point.begin(), u_of_point.end());
    image_feature_msg.channel_data[2] = ChannelData(v_of_point.begin(), v_of_point.end());
    image_feature_msg.channel_data[3] = ChannelData(velocity_x_of_point.begin(), velocity_x_of_point.end());
    image_feature_msg.channel_data[4] = ChannelData(velocity_y_of_point.begin(), velocity_y_of_point.end());

    if (g_config.feature_tracker.show_track) {
        cv::Mat tmp_img = show_img.rowRange(0, g_config.camera.row);
        cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

        for (unsigned int j = 0; j < feature_tracker_->cur_pts.size(); j++) {
            double len = std::min(1.0, 1.0 * feature_tracker_->track_cnt[j] / g_config.feature_tracker.window_size);
            cv::circle(tmp_img, feature_tracker_->cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        }

        cv::imshow("show_img", tmp_img);
        cv::waitKey(1);
    }
    
    
    return image_feature_msg;
}

MeasurementMsg MeasurementProcessor::createMeasurementMsg(int measurement_id, 
                                                         const ImageFileData& image_data) {
    MeasurementMsg msg;
    msg.measurement_id = measurement_id;
    
    // 이미지 특징점 추출
    msg.image_feature_msg = extractImageFeatures(image_data);
    
    // 현재 이미지 타임스탬프
    double current_image_timestamp = image_data.timestamp;
    
    // IMU 데이터 수집 범위 결정
    double start_time, end_time;
    
    if (prev_image_timestamp_ < 0) {
        start_time = 0;
        end_time = current_image_timestamp;
    } else {
        start_time = prev_image_timestamp_;
        end_time = current_image_timestamp;
    }
    
    // 시간 범위 내의 IMU 데이터 수집
    for (const auto& imu : imu_data_) {
        if (imu.timestamp >= start_time && imu.timestamp <= end_time) {
            IMUMsg imu_msg;
            imu_msg.timestamp = imu.timestamp;
            imu_msg.linear_acc_x = imu.linear_acc_x;
            imu_msg.linear_acc_y = imu.linear_acc_y;
            imu_msg.linear_acc_z = imu.linear_acc_z;
            imu_msg.angular_vel_x = imu.angular_vel_x;
            imu_msg.angular_vel_y = imu.angular_vel_y;
            imu_msg.angular_vel_z = imu.angular_vel_z;
            
            msg.imu_msg.push_back(imu_msg);
        }
    }
    
    // 현재 이미지 타임스탬프를 이전 타임스탬프로 저장
    prev_image_timestamp_ = current_image_timestamp;
    
    return msg;
}

void MeasurementProcessor::printDataRange() const {
    std::cout << "\n3. 데이터 범위 정보:" << std::endl;
    if (!imu_data_.empty()) {
        std::cout << "IMU 데이터 범위: ";
        printTimeInfo(imu_data_.front().timestamp);
        std::cout << " ~ ";
        printTimeInfo(imu_data_.back().timestamp);
        std::cout << std::endl;
    }
    
    if (!image_file_data_.empty()) {
        std::cout << "이미지 데이터 범위: ";
        printTimeInfo(image_file_data_.front().timestamp);
        std::cout << " ~ ";
        printTimeInfo(image_file_data_.back().timestamp);
        std::cout << std::endl;
    }
}

void MeasurementProcessor::printTimeInfo(double timestamp) const {
    std::time_t time_t = static_cast<std::time_t>(timestamp);
    std::tm* tm = std::gmtime(&time_t);
    
    std::cout << std::fixed << std::setprecision(6) 
              << timestamp << " (" 
              << std::put_time(tm, "%Y-%m-%d %H:%M:%S") << " UTC)";
} 