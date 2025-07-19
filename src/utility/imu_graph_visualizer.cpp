#include <iostream>
#include <algorithm>
#include <iomanip>
#include <sstream>

#include "utility/imu_graph_visualizer.h"

namespace utility {

IMUGraphVisualizer::IMUGraphVisualizer() 
    : running_(false), window_width_(800), window_height_(600), max_data_points_(300),
      acc_min_(-20.0), acc_max_(20.0), gyro_min_(-5.0), gyro_max_(5.0) {
}

IMUGraphVisualizer::~IMUGraphVisualizer() {
    stop();
}

bool IMUGraphVisualizer::initialize(int window_width, int window_height, int max_data_points) {
    window_width_ = window_width;
    window_height_ = window_height;
    max_data_points_ = max_data_points;
    
    graph_image_ = cv::Mat(window_height_, window_width_, CV_8UC3);
    
    return true;
}

void IMUGraphVisualizer::start() {
    if (!running_.load()) {
        running_ = true;
        visualization_thread_ = std::thread(&IMUGraphVisualizer::visualizationThread, this);
    }
}

void IMUGraphVisualizer::stop() {
    if (running_.load()) {
        running_ = false;
        if (visualization_thread_.joinable()) {
            visualization_thread_.join();
        }
        cv::destroyWindow(window_name_);
    }
}

void IMUGraphVisualizer::addIMUData(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    imu_data_.push_back({timestamp, acc, gyro});
    
    acc_x_data_.push_back(acc.x());
    acc_y_data_.push_back(acc.y());
    acc_z_data_.push_back(acc.z());
    
    gyro_x_data_.push_back(gyro.x());
    gyro_y_data_.push_back(gyro.y());
    gyro_z_data_.push_back(gyro.z());
    
    // Keep only the latest max_data_points_
    while (imu_data_.size() > max_data_points_) {
        imu_data_.pop_front();
        acc_x_data_.pop_front();
        acc_y_data_.pop_front();
        acc_z_data_.pop_front();
        gyro_x_data_.pop_front();
        gyro_y_data_.pop_front();
        gyro_z_data_.pop_front();
    }
    
    updateDataRange();
}

void IMUGraphVisualizer::updateDataRange() {
    if (acc_x_data_.empty()) return;
    
    // Update acceleration range
    double acc_min = *std::min_element(acc_x_data_.begin(), acc_x_data_.end());
    acc_min = std::min(acc_min, *std::min_element(acc_y_data_.begin(), acc_y_data_.end()));
    acc_min = std::min(acc_min, *std::min_element(acc_z_data_.begin(), acc_z_data_.end()));
    
    double acc_max = *std::max_element(acc_x_data_.begin(), acc_x_data_.end());
    acc_max = std::max(acc_max, *std::max_element(acc_y_data_.begin(), acc_y_data_.end()));
    acc_max = std::max(acc_max, *std::max_element(acc_z_data_.begin(), acc_z_data_.end()));
    
    // Update gyroscope range
    double gyro_min = *std::min_element(gyro_x_data_.begin(), gyro_x_data_.end());
    gyro_min = std::min(gyro_min, *std::min_element(gyro_y_data_.begin(), gyro_y_data_.end()));
    gyro_min = std::min(gyro_min, *std::min_element(gyro_z_data_.begin(), gyro_z_data_.end()));
    
    double gyro_max = *std::max_element(gyro_x_data_.begin(), gyro_x_data_.end());
    gyro_max = std::max(gyro_max, *std::max_element(gyro_y_data_.begin(), gyro_y_data_.end()));
    gyro_max = std::max(gyro_max, *std::max_element(gyro_z_data_.begin(), gyro_z_data_.end()));
    
    // Add some margin
    double acc_margin = (acc_max - acc_min) * 0.1;
    double gyro_margin = (gyro_max - gyro_min) * 0.1;
    
    acc_min_ = acc_min - acc_margin;
    acc_max_ = acc_max + acc_margin;
    gyro_min_ = gyro_min - gyro_margin;
    gyro_max_ = gyro_max + gyro_margin;
}

void IMUGraphVisualizer::visualizationThread() {
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    cv::moveWindow(window_name_, 100, 1000);
    
    while (running_.load()) {
        drawGraph();
        cv::imshow(window_name_, graph_image_);
        
        if (cv::waitKey(30) == 27) { // ESC key
            running_ = false;
        }
    }
}

void IMUGraphVisualizer::drawGraph() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Clear the image
    graph_image_.setTo(cv::Scalar(20, 20, 20));
    
    if (imu_data_.empty()) return;
    
    int graph_height = window_height_ / 2 - 60;
    int graph_width = window_width_ - 120;
    
    // Draw acceleration graph (top half)
    cv::rectangle(graph_image_, cv::Point(60, 30), 
                  cv::Point(60 + graph_width, 30 + graph_height), 
                  cv::Scalar(50, 50, 50), -1);
    
    // Draw gyroscope graph (bottom half)
    cv::rectangle(graph_image_, cv::Point(60, window_height_/2 + 30), 
                  cv::Point(60 + graph_width, window_height_/2 + 30 + graph_height), 
                  cv::Scalar(50, 50, 50), -1);
    
    // Draw titles
    cv::putText(graph_image_, "Acceleration (m/s^2)", cv::Point(window_width_/2 - 80, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    cv::putText(graph_image_, "Angular Velocity (rad/s)", cv::Point(window_width_/2 - 90, window_height_/2 + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    
    // Draw grid lines
    for (int i = 0; i <= 4; i++) {
        int y_acc = 30 + i * graph_height / 4;
        int y_gyro = window_height_/2 + 30 + i * graph_height / 4;
        cv::line(graph_image_, cv::Point(60, y_acc), cv::Point(60 + graph_width, y_acc), 
                 cv::Scalar(80, 80, 80), 1);
        cv::line(graph_image_, cv::Point(60, y_gyro), cv::Point(60 + graph_width, y_gyro), 
                 cv::Scalar(80, 80, 80), 1);
    }
    
    // Draw axes
    cv::line(graph_image_, cv::Point(60, 30), cv::Point(60, 30 + graph_height), 
             cv::Scalar(200, 200, 200), 2);
    cv::line(graph_image_, cv::Point(60, 30 + graph_height), cv::Point(60 + graph_width, 30 + graph_height), 
             cv::Scalar(200, 200, 200), 2);
    cv::line(graph_image_, cv::Point(60, window_height_/2 + 30), cv::Point(60, window_height_/2 + 30 + graph_height), 
             cv::Scalar(200, 200, 200), 2);
    cv::line(graph_image_, cv::Point(60, window_height_/2 + 30 + graph_height), 
             cv::Point(60 + graph_width, window_height_/2 + 30 + graph_height), 
             cv::Scalar(200, 200, 200), 2);
    
    // Draw data
    drawTimeSeries(graph_image_, acc_x_data_, acc_x_color_, 0, acc_min_, acc_max_, "acc_x");
    drawTimeSeries(graph_image_, acc_y_data_, acc_y_color_, 0, acc_min_, acc_max_, "acc_y");
    drawTimeSeries(graph_image_, acc_z_data_, acc_z_color_, 0, acc_min_, acc_max_, "acc_z");
    
    drawTimeSeries(graph_image_, gyro_x_data_, gyro_x_color_, 1, gyro_min_, gyro_max_, "gyro_x");
    drawTimeSeries(graph_image_, gyro_y_data_, gyro_y_color_, 1, gyro_min_, gyro_max_, "gyro_y");
    drawTimeSeries(graph_image_, gyro_z_data_, gyro_z_color_, 1, gyro_min_, gyro_max_, "gyro_z");
    
    // Draw legend
    int legend_y = 10;
    cv::rectangle(graph_image_, cv::Point(window_width_ - 150, legend_y), 
                  cv::Point(window_width_ - 10, legend_y + 160), cv::Scalar(40, 40, 40), -1);
    
    cv::putText(graph_image_, "acc_x", cv::Point(window_width_ - 140, legend_y + 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, acc_x_color_, 1);
    cv::putText(graph_image_, "acc_y", cv::Point(window_width_ - 140, legend_y + 40), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, acc_y_color_, 1);
    cv::putText(graph_image_, "acc_z", cv::Point(window_width_ - 140, legend_y + 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, acc_z_color_, 1);
    cv::putText(graph_image_, "gyro_x", cv::Point(window_width_ - 140, legend_y + 90), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, gyro_x_color_, 1);
    cv::putText(graph_image_, "gyro_y", cv::Point(window_width_ - 140, legend_y + 110), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, gyro_y_color_, 1);
    cv::putText(graph_image_, "gyro_z", cv::Point(window_width_ - 140, legend_y + 130), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, gyro_z_color_, 1);
    
    // Draw scale labels
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1);
    
    // Acceleration scale
    for (int i = 0; i <= 4; i++) {
        double val = acc_min_ + (acc_max_ - acc_min_) * (4 - i) / 4.0;
        ss.str("");
        ss << val;
        cv::putText(graph_image_, ss.str(), cv::Point(5, 35 + i * graph_height / 4), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(200, 200, 200), 1);
    }
    
    // Gyroscope scale
    for (int i = 0; i <= 4; i++) {
        double val = gyro_min_ + (gyro_max_ - gyro_min_) * (4 - i) / 4.0;
        ss.str("");
        ss << val;
        cv::putText(graph_image_, ss.str(), cv::Point(5, window_height_/2 + 35 + i * graph_height / 4), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(200, 200, 200), 1);
    }
}

void IMUGraphVisualizer::drawTimeSeries(cv::Mat& img, const std::deque<double>& data, 
                                       const cv::Scalar& color, int graph_idx, 
                                       double min_val, double max_val,
                                       const std::string& label) {
    if (data.size() < 2) return;
    
    int graph_height = window_height_ / 2 - 60;
    int graph_width = window_width_ - 120;
    int graph_y_offset = (graph_idx == 0) ? 30 : (window_height_/2 + 30);
    
    std::vector<cv::Point> points;
    for (size_t i = 0; i < data.size(); i++) {
        int x = 60 + (i * graph_width) / (max_data_points_ - 1);
        double normalized = (data[i] - min_val) / (max_val - min_val);
        normalized = std::max(0.0, std::min(1.0, normalized));
        int y = graph_y_offset + graph_height - (int)(normalized * graph_height);
        points.push_back(cv::Point(x, y));
    }
    
    // Draw the line
    for (size_t i = 1; i < points.size(); i++) {
        cv::line(img, points[i-1], points[i], color, 2);
    }
    
    // Draw current value
    if (!data.empty()) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << data.back();
        cv::putText(img, ss.str(), cv::Point(points.back().x + 5, points.back().y), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 1);
    }
}

} // namespace utility