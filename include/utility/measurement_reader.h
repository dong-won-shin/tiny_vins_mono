#ifndef MEASUREMENT_READER_H
#define MEASUREMENT_READER_H

#include <array>
#include <string>
#include <vector>

struct IMUMsg {
  double timestamp;
  double linear_acc_x, linear_acc_y, linear_acc_z;
  double angular_vel_x, angular_vel_y, angular_vel_z;
};

struct Point3D {
  int index;
  double x, y, z;
};

using ChannelData = std::vector<double>;

struct ImageFeatureMsg {
  double timestamp;
  std::string frame_id;
  int points_count;
  int channels_count;
  std::vector<Point3D> feature_points;
  std::array<ChannelData, 5> channel_data;
};

struct MeasurementMsg {
  int measurement_id;
  std::vector<IMUMsg> imu_msg;
  ImageFeatureMsg image_feature_msg;
};

bool readMeasurementFile(const std::string& filepath, MeasurementMsg& measurement);

void printMeasurementMsg(const MeasurementMsg& measurement);

#endif  // MEASUREMENT_READER_H