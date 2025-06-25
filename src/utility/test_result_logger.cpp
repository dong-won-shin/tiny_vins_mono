#include "utility/test_result_logger.h"

#include <experimental/filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace utility;
namespace fs = std::experimental::filesystem;

TestResultLogger::TestResultLogger() : log_dir_("") {}

TestResultLogger::~TestResultLogger() {}

bool TestResultLogger::initialize(const std::string& config_path) {
  try {
    std::cout << "=== TestResultLogger Initialization Start ===" << std::endl;

    // Create log directory with timestamp
    log_dir_ = "logs/" + getTimestampString();
    if (fs::create_directories(log_dir_)) {
      std::cout << "Log directory created: " << log_dir_ << std::endl;
    } else {
      std::cout << "Log directory already exists or failed to create: " << log_dir_ << std::endl;
    }

    // Copy config file to log directory
    copyConfigFile(config_path);

    std::cout << "=== TestResultLogger Initialization Complete ===" << std::endl;
    return true;

  } catch (const std::exception& e) {
    std::cerr << "TestResultLogger init failed: " << e.what() << std::endl;
    return false;
  }
}

void TestResultLogger::addPose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation,
                               double timestamp) {
  std::lock_guard<std::mutex> lock(pose_mutex_);

  // Validate input data
  if (!position.allFinite() || !rotation.allFinite()) {
    std::cerr << "TestResultLogger: Invalid pose data received!" << std::endl;
    return;
  }

  camera_poses_.push_back(position);
  camera_rotations_.push_back(rotation);
  trajectory_timestamps_.push_back(timestamp);

  // Log progress occasionally
  static int log_counter = 0;
  if (++log_counter % 10 == 0) {
    std::cout << "TestResultLogger received pose " << camera_poses_.size() << ": [" << position.x()
              << ", " << position.y() << ", " << position.z() << "] timestamp: " << timestamp
              << std::endl;
  }
}

void TestResultLogger::saveTrajectoryToFile() {
  std::lock_guard<std::mutex> lock(pose_mutex_);

  if (camera_poses_.empty() || trajectory_timestamps_.empty() || camera_rotations_.empty()) {
    std::cout << "No trajectory data to save." << std::endl;
    return;
  }

  try {
    if (log_dir_.empty()) {
      std::cerr << "Log directory not initialized. Cannot save trajectory." << std::endl;
      return;
    }

    std::string file_path = log_dir_ + "/trajectory_pose.txt";
    std::ofstream pose_file(file_path);

    if (!pose_file.is_open()) {
      std::cerr << "Failed to open " << file_path << " for writing" << std::endl;
      return;
    }

    pose_file << "# timestamp tx ty tz qx qy qz qw" << std::endl;

    // Ensure we don't exceed the bounds
    size_t num_poses =
        std::min({camera_poses_.size(), camera_rotations_.size(), trajectory_timestamps_.size()});

    for (size_t i = 0; i < num_poses; i++) {
      const auto& pos = camera_poses_[i];
      const auto& rot = camera_rotations_[i];
      double timestamp = trajectory_timestamps_[i];

      if (pos.allFinite() && rot.allFinite()) {
        Eigen::Quaterniond q(rot);
        pose_file << std::fixed << std::setprecision(9) << timestamp << " " << std::fixed
                  << std::setprecision(6) << pos.x() << " " << pos.y() << " " << pos.z() << " "
                  << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      }
    }

    pose_file.close();
    std::cout << "✅ Trajectory saved to " << file_path << " (" << num_poses << " poses)"
              << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "Error saving trajectory to file: " << e.what() << std::endl;
  }
}

size_t TestResultLogger::getPoseCount() const {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return camera_poses_.size();
}

std::string TestResultLogger::getTimestampString() const {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
  return ss.str();
}

void TestResultLogger::copyConfigFile(const std::string& config_path) {
  try {
    fs::path src_path(config_path);
    if (fs::exists(src_path)) {
      fs::path dst_path = fs::path(log_dir_) / src_path.filename();
      fs::copy_file(src_path, dst_path, fs::copy_options::overwrite_existing);
      std::cout << "✅ Copied config file to " << dst_path << std::endl;
    } else {
      std::cerr << "⚠️ Config file not found at " << config_path << ", skipping copy." << std::endl;
    }
  } catch (const fs::filesystem_error& e) {
    std::cerr << "❌ Failed to copy config file: " << e.what() << std::endl;
  }
}