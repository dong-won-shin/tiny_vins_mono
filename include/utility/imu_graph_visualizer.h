#ifndef UTILITY_IMU_GRAPH_VISUALIZER_H
#define UTILITY_IMU_GRAPH_VISUALIZER_H

#include <opencv2/opencv.hpp>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <Eigen/Dense>

namespace utility {

class IMUGraphVisualizer {
public:
    IMUGraphVisualizer();
    ~IMUGraphVisualizer();

    bool initialize(int window_width = 800, int window_height = 600, int max_data_points = 300);
    void start();
    void stop();

    void addIMUData(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro);
    
    bool isRunning() const { return running_.load(); }

private:
    struct IMUDataPoint {
        double timestamp;
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
    };

    void visualizationThread();
    void drawGraph();
    void drawTimeSeries(cv::Mat& img, const std::deque<double>& data, 
                       const cv::Scalar& color, int graph_idx, double min_val, double max_val,
                       const std::string& label);
    void updateDataRange();

    std::deque<IMUDataPoint> imu_data_;
    std::deque<double> acc_x_data_, acc_y_data_, acc_z_data_;
    std::deque<double> gyro_x_data_, gyro_y_data_, gyro_z_data_;
    
    std::mutex data_mutex_;
    std::thread visualization_thread_;
    std::atomic<bool> running_;
    
    int window_width_;
    int window_height_;
    int max_data_points_;
    
    double acc_min_, acc_max_;
    double gyro_min_, gyro_max_;
    
    cv::Mat graph_image_;
    
    const std::string window_name_ = "IMU Data Visualization";
    
    const cv::Scalar acc_x_color_ = cv::Scalar(255, 0, 0);      // Blue
    const cv::Scalar acc_y_color_ = cv::Scalar(0, 255, 0);      // Green  
    const cv::Scalar acc_z_color_ = cv::Scalar(0, 0, 255);      // Red
    const cv::Scalar gyro_x_color_ = cv::Scalar(255, 255, 0);   // Cyan
    const cv::Scalar gyro_y_color_ = cv::Scalar(255, 0, 255);   // Magenta
    const cv::Scalar gyro_z_color_ = cv::Scalar(0, 255, 255);   // Yellow
};

} // namespace utility

#endif // UTILITY_IMU_GRAPH_VISUALIZER_H