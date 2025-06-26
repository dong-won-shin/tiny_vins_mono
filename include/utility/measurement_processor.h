#ifndef UTILITY__MEASUREMENT_PROCESSOR_H
#define UTILITY__MEASUREMENT_PROCESSOR_H

#include <array>
#include <string>
#include <vector>

#include "common/image_frame.h"
#include "frontend/feature_tracker.h"

namespace utility {

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

struct ImageFileData {
    double timestamp;
    std::string filename;
    std::string full_path;
};

class MeasurementProcessor {
public:
    MeasurementProcessor();
    ~MeasurementProcessor();

    // Initialization
    bool initialize(const std::string& imu_filepath, const std::string& image_csv_filepath,
                    const std::string& image_dir, const std::string& config_filepath);

    // Data loading
    bool loadImuData(const std::string& filepath);
    bool loadImageFileData(const std::string& csv_filepath, const std::string& image_dir);

    // Create MeasurementMsg
    MeasurementMsg createMeasurementMsg(int measurement_id, const ImageFileData& image_data);

    // Data accessors
    const std::vector<IMUMsg>& getIMUData() const {
        return imu_data_;
    }
    const std::vector<ImageFileData>& getImageFileData() const {
        return image_file_data_;
    }

    // Utility functions
    void printDataRange() const;
    void printTimeInfo(double timestamp) const;

private:
    // Member variables
    std::vector<IMUMsg> imu_data_;
    std::vector<ImageFileData> image_file_data_;
    std::vector<double> image_timestamps_;
    std::vector<std::string> image_files_;

    // Store previous image timestamp
    double prev_image_timestamp_;

  // Feature tracker related
  std::unique_ptr<frontend::FeatureTracker> feature_tracker_;

    // Extract image features
    ImageFeatureMsg extractImageFeatures(const ImageFileData& image_data);

    // Clean filename (remove whitespace and newlines)
    std::string cleanFilename(const std::string& filename);
};

}  // namespace utility

#endif  // UTILITY__MEASUREMENT_PROCESSOR_H
