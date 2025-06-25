#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <pangolin/pangolin.h>

#include <Eigen/Dense>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace utility {

class Visualizer {
public:
    Visualizer();
    ~Visualizer();

    // Initialize and start the viewer
    bool initialize();
    bool start();
    void stop();

    // Update camera pose and trajectory
    void updateCameraPose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation, double timestamp);

    // Update 3D feature points
    void updateFeaturePoints3D(const std::vector<Eigen::Vector3d>& points);

    // Check if viewer is running
    bool isRunning() const {
        return running_;
    }

    // Test function for debugging
    void testWithSimpleData();

    void waitUntilRenderReady();

    void pangolinViewerThread();

private:
    // Pangolin viewer variables
    std::vector<Eigen::Vector3d> camera_poses_;
    std::vector<Eigen::Matrix3d> camera_rotations_;
    std::vector<double> trajectory_timestamps_;
    std::vector<Eigen::Vector3d> feature_points_3d_;

    // Threading
    std::thread viewer_thread_;
    std::mutex pose_mutex_;
    std::mutex points_mutex_;
    bool running_;

    // Pangolin objects
    pangolin::OpenGlRenderState* g_s_cam_;
    pangolin::View* g_d_cam_;

    // UI control variables
    pangolin::Var<bool>* show_grid_;
    pangolin::Var<bool>* show_trajectory_;
    pangolin::Var<bool>* show_points_;
    pangolin::Var<std::string>* status_;

    // Private methods
    void setupPangolinViewer();
    void drawCameraTrajectory();
    void drawFeaturePoints3D();
    void drawCoordinateFrame();
    void drawGrid();
    void drawCameraFrustum(const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot, double size = 0.1);
    void adjustCameraViewToFitTrajectory();

    std::condition_variable render_ready_cv_;
    std::mutex render_ready_mutex_;
    bool render_ready_ = false;
};

}  // namespace utility

#endif  // VISUALIZER_H