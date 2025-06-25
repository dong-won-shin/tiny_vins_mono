#ifndef TEST_RESULT_LOGGER_H
#define TEST_RESULT_LOGGER_H

#include <Eigen/Dense>
#include <chrono>
#include <experimental/filesystem>
#include <mutex>
#include <string>
#include <vector>

namespace utility {

class TestResultLogger {
public:
  TestResultLogger();
  ~TestResultLogger();

  // Initialize logger with config file path
  bool initialize(const std::string& config_path);

  // Add pose data to trajectory
  void addPose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation, double timestamp);

  // Save trajectory to file
  void saveTrajectoryToFile();

  // Get current trajectory data
  size_t getPoseCount() const;

  // Check if logger is initialized
  bool isInitialized() const { return !log_dir_.empty(); }

  // Get log directory path
  std::string getLogDirectory() const { return log_dir_; }

private:
  // Trajectory data
  std::vector<Eigen::Vector3d> camera_poses_;
  std::vector<Eigen::Matrix3d> camera_rotations_;
  std::vector<double> trajectory_timestamps_;

  // Thread safety
  mutable std::mutex pose_mutex_;

  // Log directory
  std::string log_dir_;

  // Helper functions
  std::string getTimestampString() const;
  void copyConfigFile(const std::string& config_path);
};

}  // namespace utility

#endif  // TEST_RESULT_LOGGER_H