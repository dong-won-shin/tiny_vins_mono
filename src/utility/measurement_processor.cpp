#include "utility/measurement_processor.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <set>
#include <sstream>

#include "utility/config.h"

MeasurementProcessor::MeasurementProcessor() : prev_image_timestamp_(-1.0) {}

MeasurementProcessor::~MeasurementProcessor() {}

bool MeasurementProcessor::initialize(const std::string& imu_filepath,
                                      const std::string& image_csv_filepath,
                                      const std::string& image_dir,
                                      const std::string& config_filepath) {
  std::cout << "=== MeasurementProcessor Initialization ===" << std::endl;

  // Load configuration
  if (!g_config.loadFromYaml(config_filepath)) {
    std::cout << "Failed to load config from " << config_filepath << std::endl;
    return false;
  }

  // Print configuration for debugging
  g_config.print();

  // Initialize feature tracker
  feature_tracker_.reset(new feature_tracker::FeatureTracker());
  feature_tracker_->readIntrinsicParameter(config_filepath);

  // Load data
  if (!loadImuData(imu_filepath)) {
    std::cerr << "Failed to load IMU data" << std::endl;
    return false;
  }

  if (!loadImageFileData(image_csv_filepath, image_dir)) {
    std::cerr << "Failed to load image data" << std::endl;
    return false;
  }

  // Print data range
  printDataRange();

  return true;
}

bool MeasurementProcessor::loadImuData(const std::string& filepath) {
  std::cout << "\n1. Loading IMU data..." << std::endl;

  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Cannot open IMU file: " << filepath << std::endl;
    return false;
  }

  std::string line;
  // Skip header
  std::getline(file, line);

  while (std::getline(file, line)) {
    if (line.empty()) continue;

    std::istringstream iss(line);
    std::string token;
    IMUMsg data;

    // Convert timestamp [ns] to seconds
    if (std::getline(iss, token, ',')) {
      data.timestamp = std::stod(token) / 1e9;  // Convert ns to seconds
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
  std::cout << "Loaded " << imu_data_.size() << " IMU data entries" << std::endl;
  return true;
}

bool MeasurementProcessor::loadImageFileData(const std::string& csv_filepath,
                                             const std::string& image_dir) {
  std::cout << "\n2. Loading image data..." << std::endl;

  std::ifstream file(csv_filepath);
  if (!file.is_open()) {
    std::cerr << "Cannot open image CSV file: " << csv_filepath << std::endl;
    return false;
  }

  std::string line;
  // Skip header
  std::getline(file, line);

  while (std::getline(file, line)) {
    if (line.empty()) continue;

    std::istringstream iss(line);
    std::string token;
    ImageFileData data;

    // Convert timestamp [ns] to seconds
    if (std::getline(iss, token, ',')) {
      data.timestamp = std::stod(token) / 1e9;  // Convert ns to seconds
    }

    // filename
    if (std::getline(iss, token)) {
      data.filename = cleanFilename(token);
      data.full_path = image_dir + "/" + data.filename;
    }

    image_file_data_.push_back(data);
  }

  file.close();
  std::cout << "Loaded " << image_file_data_.size() << " image data entries" << std::endl;
  return true;
}

std::string MeasurementProcessor::cleanFilename(const std::string& filename) {
  std::string cleaned = filename;
  // Remove leading and trailing whitespace and newlines
  cleaned.erase(cleaned.find_last_not_of(" \n\r\t") + 1);
  cleaned.erase(0, cleaned.find_first_not_of(" \n\r\t"));
  return cleaned;
}

ImageFeatureMsg MeasurementProcessor::extractImageFeatures(const ImageFileData& image_data) {
  ImageFeatureMsg image_feature_msg;

  // Always set timestamp and frame_id (regardless of PUB_THIS_FRAME)
  image_feature_msg.timestamp = image_data.timestamp;
  image_feature_msg.frame_id = image_data.filename;

  cv::Mat show_img = cv::imread(image_data.full_path, cv::IMREAD_GRAYSCALE);
  if (show_img.empty()) {
    std::cerr << "Cannot load image: " << image_data.full_path << std::endl;
    return image_feature_msg;
  }

  feature_tracker_->readImage(show_img.rowRange(0, g_config.camera.row), image_data.timestamp);

  for (unsigned int i = 0;; i++) {
    bool completed = false;
    completed |= feature_tracker_->updateID(i);
    if (!completed) break;
  }

  std::vector<int32_t> id_of_point;
  std::vector<float> u_of_point;
  std::vector<float> v_of_point;
  std::vector<float> velocity_x_of_point;
  std::vector<float> velocity_y_of_point;

  std::set<int> hash_ids;
  auto& un_pts = feature_tracker_->cur_un_pts;
  auto& cur_pts = feature_tracker_->cur_pts;
  auto& ids = feature_tracker_->ids;
  auto& pts_velocity = feature_tracker_->pts_velocity;

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
  image_feature_msg.channel_data[3] =
      ChannelData(velocity_x_of_point.begin(), velocity_x_of_point.end());
  image_feature_msg.channel_data[4] =
      ChannelData(velocity_y_of_point.begin(), velocity_y_of_point.end());

  if (g_config.feature_tracker.show_track) {
    cv::Mat tmp_img = show_img.rowRange(0, g_config.camera.row);
    cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

    for (unsigned int j = 0; j < feature_tracker_->cur_pts.size(); j++) {
      double len = std::min(
          1.0, 1.0 * feature_tracker_->track_cnt[j] / g_config.feature_tracker.window_size);
      cv::circle(tmp_img, feature_tracker_->cur_pts[j], 2,
                 cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }

    cv::imshow("tracking image", tmp_img);
    cv::waitKey(1);
  }

  return image_feature_msg;
}

MeasurementMsg MeasurementProcessor::createMeasurementMsg(int measurement_id,
                                                          const ImageFileData& image_data) {
  MeasurementMsg msg;
  msg.measurement_id = measurement_id;

  // Extract image features
  msg.image_feature_msg = extractImageFeatures(image_data);

  // Current image timestamp
  double current_image_timestamp = image_data.timestamp;

  // Determine IMU data collection range
  double start_time, end_time;

  if (prev_image_timestamp_ < 0) {
    start_time = 0;
    end_time = current_image_timestamp;
  } else {
    start_time = prev_image_timestamp_;
    end_time = current_image_timestamp;
  }

  // Collect IMU data within time range
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

  // Store current image timestamp as previous timestamp
  prev_image_timestamp_ = current_image_timestamp;

  return msg;
}

void MeasurementProcessor::printDataRange() const {
  std::cout << "\n3. Data range information:" << std::endl;
  if (!imu_data_.empty()) {
    std::cout << "IMU data range: ";
    printTimeInfo(imu_data_.front().timestamp);
    std::cout << " ~ ";
    printTimeInfo(imu_data_.back().timestamp);
    std::cout << std::endl;
  }

  if (!image_file_data_.empty()) {
    std::cout << "Image data range: ";
    printTimeInfo(image_file_data_.front().timestamp);
    std::cout << " ~ ";
    printTimeInfo(image_file_data_.back().timestamp);
    std::cout << std::endl;
  }
}

void MeasurementProcessor::printTimeInfo(double timestamp) const {
  std::time_t time_t = static_cast<std::time_t>(timestamp);
  std::tm* tm = std::gmtime(&time_t);

  std::cout << std::fixed << std::setprecision(6) << timestamp << " ("
            << std::put_time(tm, "%Y-%m-%d %H:%M:%S") << " UTC)";
}