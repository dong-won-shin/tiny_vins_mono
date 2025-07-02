#ifndef VIO_SYSTEM_H
#define VIO_SYSTEM_H

#include <memory>
#include <thread>
#include <vector>
#include <Eigen/Dense>

#include "backend/estimator.h"
#include "utility/config.h"
#include "utility/measurement_processor.h"
#include "utility/test_result_logger.h"
#include "utility/visualizer.h"

class VIOSystem {
public:
    VIOSystem(std::shared_ptr<utility::Config> config);
    ~VIOSystem();

    bool initialize();
    void processSequence();
    void shutdown();

private:
    // Core components
    std::shared_ptr<utility::Config> config_;
    std::unique_ptr<MeasurementProcessor> measurement_processor_;
    std::unique_ptr<backend::Estimator> vio_estimator_;
    std::unique_ptr<Visualizer> visualizer_;
    std::unique_ptr<utility::TestResultLogger> result_logger_;

    // Processing thread
    std::unique_ptr<std::thread> vio_process_thread_;

    // Private methods
    void vioInitialize();
    void vioProcess();
    void onFrameProcessed(const utility::MeasurementMsg& measurement, double& current_time, int32_t measurement_id);
    void onSequenceComplete();
    
    void processIMUData(const std::vector<utility::IMUMsg>& imu_msg, const utility::ImageFeatureMsg& image_msg, double& current_time);
    void processImageData(const utility::ImageFeatureMsg& image_msg);
    void updateVisualization(double timestamp);
    void updateCameraPose(double timestamp);
    void updateFeaturePoints3D();
    
    // IMU data processing helpers
    Eigen::Vector3d extractAcceleration(const utility::IMUMsg& imu_data);
    Eigen::Vector3d extractAngularVelocity(const utility::IMUMsg& imu_data);
    void interpolateIMUData(const Eigen::Vector3d& prev_acc, const Eigen::Vector3d& prev_gyro,
                           const utility::IMUMsg& current_imu,
                           double dt1, double dt2,
                           Eigen::Vector3d& interp_acc, Eigen::Vector3d& interp_gyro);
};

#endif // VIO_SYSTEM_H