#include <algorithm>
#include <chrono>
#include <experimental/filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "utility/config.h"
#include "utility/visualizer.h"

namespace utility {
namespace fs = std::experimental::filesystem;

Visualizer::Visualizer()
    : running_(false),
      g_s_cam_(nullptr),
      g_d_cam_(nullptr),
      show_grid_(nullptr),
      show_trajectory_(nullptr),
      show_points_(nullptr),
      status_(nullptr) {}

Visualizer::~Visualizer() {
    stop();

    // Clean up UI variables
    if (show_grid_)
        delete show_grid_;
    if (show_trajectory_)
        delete show_trajectory_;
    if (show_points_)
        delete show_points_;
    if (status_)
        delete status_;
}

bool Visualizer::initialize() {
    try {
        std::cout << "=== Visualizer Initialization Start ===" << std::endl;

        // DISPLAY environment variable check
        const char* display = getenv("DISPLAY");
        std::cout << "DISPLAY: " << (display ? display : "NOT SET") << std::endl;

        // Window dimensions
        const int window_width = 1024;
        const int window_height = 768;

        // Create Pangolin window
        std::cout << "Creating Pangolin window..." << std::endl;
        pangolin::CreateWindowAndBind("Tiny VINS Mono - 3D Trajectory Viewer", window_width, window_height);
        std::cout << "Window created successfully" << std::endl;

        // OpenGL settings
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        std::cout << "OpenGL depth test and blend enabled" << std::endl;

        // Render state setup with better projection matrix
        // Parameters: width, height, fx, fy, cx, cy, near, far
        // Using more appropriate values for 3D visualization
        g_s_cam_ = new pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(window_width, window_height, 420, 420,  // focal length (fx, fy)
                                       window_width / 2, window_height / 2,    // principal point (cx, cy)
                                       0.1, 1000),                             // near, far planes
            pangolin::ModelViewLookAt(-3, 0, 3,                                // camera position
                                      0, 0, 0,                                 // look at point
                                      pangolin::AxisZ));                       // up vector
        std::cout << "Render state created" << std::endl;

        // View setup - use full window for 3D view
        g_d_cam_ = &pangolin::CreateDisplay()
                        .SetBounds(0.0, 1.0, 0.0, 1.0, (double)window_width / window_height)
                        .SetHandler(new pangolin::Handler3D(*g_s_cam_));
        std::cout << "Display created" << std::endl;

        // Add UI panel for controls
        pangolin::CreatePanel("ui").SetBounds(0.0, 0.3, 0.0, 0.2);

        // Add some UI variables for controls
        show_grid_ = new pangolin::Var<bool>("ui.Show Grid", true, true);
        show_trajectory_ = new pangolin::Var<bool>("ui.Show Trajectory", true, true);
        show_points_ = new pangolin::Var<bool>("ui.Show 3D Points", true, true);
        status_ = new pangolin::Var<std::string>("ui.Status", "Ready");

        std::cout << "=== Visualizer Initialization Complete ===" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Visualizer init failed: " << e.what() << std::endl;
        return false;
    }
}

bool Visualizer::start() {
    if (!running_) {
        try {
            running_ = true;
            viewer_thread_ = std::thread(&Visualizer::pangolinViewerThread, this);
            std::cout << "Visualizer started successfully" << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to start Visualizer: " << e.what() << std::endl;
            running_ = false;
            return false;
        }
    }
    return true;  // Consider as success if already running
}

void Visualizer::stop() {
    running_ = false;
    if (viewer_thread_.joinable()) {
        viewer_thread_.join();
    }
}

void Visualizer::setupPangolinViewer() {
    // This function is called from pangolinViewerThread
    // Initialization is already done in initialize()
}

void Visualizer::pangolinViewerThread() {
    running_ = true;
    try {
        {
            std::lock_guard<std::mutex> lk(render_ready_mutex_);
            render_ready_ = true;
        }
        render_ready_cv_.notify_all();
        std::cout << "=== Starting Visualizer render loop ===" << std::endl;

        // Check state before starting loop
        std::cout << "Before loop - running_: " << (running_ ? "true" : "false") << std::endl;
        std::cout << "Before loop - ShouldQuit(): " << (pangolin::ShouldQuit() ? "true" : "false") << std::endl;

        int frame_count = 0;
        while (!pangolin::ShouldQuit() && running_) {
            frame_count++;

            // Log output for every frame (first 10 frames)
            if (frame_count <= 10) {
                std::cout << "Render frame " << frame_count << " - poses: " << camera_poses_.size()
                          << ", running: " << (running_ ? "true" : "false")
                          << ", should_quit: " << (pangolin::ShouldQuit() ? "true" : "false") << std::endl;
            }
            // Then output every 10 frames
            else if (frame_count % 10 == 0) {
                std::cout << "Render frame " << frame_count << " - poses: " << camera_poses_.size() << std::endl;
            }

            // Clear screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // Activate camera view
            if (g_d_cam_ && g_s_cam_) {
                g_d_cam_->Activate(*g_s_cam_);
            } else {
                std::cerr << "Warning: Camera view not available at frame " << frame_count << std::endl;
            }

            // Draw coordinate axes
            drawCoordinateFrame();

            // Draw grid if enabled
            if (show_grid_ && *show_grid_) {
                drawGrid();
            }

            // Draw trajectory if enabled
            if (show_trajectory_ && *show_trajectory_) {
                drawCameraTrajectory();
            }

            // Draw 3D map points if enabled
            if (show_points_ && *show_points_) {
                drawFeaturePoints3D();
            }

            // Update status
            if (status_) {
                std::stringstream ss;
                ss << "Poses: " << camera_poses_.size() << " | Points: " << feature_points_3d_.size();
                *status_ = ss.str();
            }

            // Complete frame
            pangolin::FinishFrame();

            // Control frame rate
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }

        std::cout << "=== Visualizer render loop ended ===" << std::endl;
        std::cout << "Final frame count: " << frame_count << std::endl;
        std::cout << "Final pose count: " << camera_poses_.size() << std::endl;
        std::cout << "Exit reason - running_: " << (running_ ? "true" : "false") << std::endl;
        std::cout << "Exit reason - ShouldQuit(): " << (pangolin::ShouldQuit() ? "true" : "false") << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Visualizer render error: " << e.what() << std::endl;
    }
}

void Visualizer::drawCoordinateFrame() {
    static int frame_count = 0;
    frame_count++;

    // Log output only for first 5 frames
    if (frame_count <= 5) {
        std::cout << "Drawing coordinate frame - frame " << frame_count << std::endl;
    }

    // Draw coordinate frame at origin with larger size for better visibility
    glLineWidth(4.0f);

    // X axis - red
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glEnd();

    // Y axis - green
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glEnd();

    // Z axis - blue
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    glEnd();

    if (frame_count <= 5) {
        std::cout << "Coordinate frame drawn successfully" << std::endl;
    }
}

void Visualizer::drawGrid() {
    // Draw XY plane grid with larger size for better visibility
    glColor3f(0.4f, 0.4f, 0.4f);  // Slightly brighter gray color for grid
    glLineWidth(1.5f);

    const float grid_size = 20.0f;  // Grid extends from -10 to +10 in both X and Y
    const float grid_step = 1.0f;   // Grid spacing (larger for better visibility)
    const int num_lines = (int)(grid_size / grid_step) + 1;

    glBegin(GL_LINES);
    // Draw vertical lines (parallel to Y axis)
    for (int i = 0; i < num_lines; i++) {
        float x = -grid_size / 2 + i * grid_step;
        glVertex3f(x, -grid_size / 2, 0.0f);
        glVertex3f(x, grid_size / 2, 0.0f);
    }
    // Draw horizontal lines (parallel to X axis)
    for (int i = 0; i < num_lines; i++) {
        float y = -grid_size / 2 + i * grid_step;
        glVertex3f(-grid_size / 2, y, 0.0f);
        glVertex3f(grid_size / 2, y, 0.0f);
    }
    glEnd();
}

void Visualizer::drawCameraTrajectory() {
    std::lock_guard<std::mutex> lock(pose_mutex_);

    static int draw_count = 0;
    draw_count++;

    if (camera_poses_.empty()) {
        static int empty_count = 0;
        if (++empty_count % 10 == 0) {
            std::cout << "[drawCameraTrajectory] No poses available (count: " << empty_count << ")" << std::endl;
        }
        return;
    }

    // Detailed log output only for first 5 times
    if (draw_count <= 5) {
        std::cout << "[drawCameraTrajectory] Drawing " << camera_poses_.size() << " poses (count: " << draw_count << ")"
                  << std::endl;
        std::cout << "First pose: [" << camera_poses_[0].x() << ", " << camera_poses_[0].y() << ", "
                  << camera_poses_[0].z() << "]" << std::endl;
        std::cout << "Last pose: [" << camera_poses_.back().x() << ", " << camera_poses_.back().y() << ", "
                  << camera_poses_.back().z() << "]" << std::endl;
    }

    // Draw camera trajectory
    glColor3f(0.0f, 1.0f, 0.0f);  // Green color for trajectory
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (const auto& pose : camera_poses_) {
        if (pose.allFinite()) {
            glVertex3f(pose.x(), pose.y(), pose.z());
        }
    }
    glEnd();

    // Draw camera positions
    glPointSize(3.0f);
    glColor3f(1.0f, 0.0f, 0.0f);  // Red color for camera positions
    glBegin(GL_POINTS);
    for (const auto& pose : camera_poses_) {
        if (pose.allFinite()) {
            glVertex3f(pose.x(), pose.y(), pose.z());
        }
    }
    glEnd();

    glFlush();
    glFinish();

    if (draw_count <= 5) {
        std::cout << "Trajectory drawn successfully" << std::endl;
    }

    try {
        // Debug: Start drawing trajectory
        static int draw_count = 0;
        if (++draw_count % 100 == 0) {
            std::cout << "drawCameraTrajectory: Drawing " << camera_poses_.size() << " poses (count: " << draw_count
                      << ")" << std::endl;
        }

        // Draw camera trajectory
        glColor3f(0.0f, 1.0f, 0.0f);  // Green color for trajectory
        glLineWidth(2.0f);
        glBegin(GL_LINE_STRIP);
        for (const auto& pose : camera_poses_) {
            if (pose.allFinite()) {
                glVertex3f(pose.x(), pose.y(), pose.z());
            }
        }
        glEnd();

        // Draw camera positions
        glPointSize(3.0f);
        glColor3f(1.0f, 0.0f, 0.0f);  // Red color for camera positions
        glBegin(GL_POINTS);
        for (const auto& pose : camera_poses_) {
            if (pose.allFinite()) {
                glVertex3f(pose.x(), pose.y(), pose.z());
            }
        }
        glEnd();

        // Draw camera frustums (every 10th camera to avoid clutter)
        if (camera_poses_.size() == camera_rotations_.size()) {
            for (size_t i = 0; i < camera_poses_.size(); i += 10) {
                if (camera_poses_[i].allFinite() && camera_rotations_[i].allFinite()) {
                    drawCameraFrustum(camera_poses_[i], camera_rotations_[i], 0.1);
                }
            }
        }

        // Draw current camera frame and frustum
        if (!camera_poses_.empty() && !camera_rotations_.empty()) {
            Eigen::Vector3d current_pos = camera_poses_.back();
            Eigen::Matrix3d current_rot = camera_rotations_.back();

            if (current_pos.allFinite() && current_rot.allFinite()) {
                // Draw current camera frustum (larger and more visible)
                glColor3f(1.0f, 1.0f, 0.0f);  // Bright yellow for current camera
                glLineWidth(2.5f);
                drawCameraFrustum(current_pos, current_rot, 0.15);

                // Camera frame axes
                double frame_size = 0.08;

                // X axis - red
                glColor3f(1.0f, 0.0f, 0.0f);
                glLineWidth(3.0f);
                glBegin(GL_LINES);
                glVertex3f(current_pos.x(), current_pos.y(), current_pos.z());
                Eigen::Vector3d x_end = current_pos + current_rot.col(0) * frame_size;
                if (x_end.allFinite()) {
                    glVertex3f(x_end.x(), x_end.y(), x_end.z());
                }
                glEnd();

                // Y axis - green
                glColor3f(0.0f, 1.0f, 0.0f);
                glBegin(GL_LINES);
                glVertex3f(current_pos.x(), current_pos.y(), current_pos.z());
                Eigen::Vector3d y_end = current_pos + current_rot.col(1) * frame_size;
                if (y_end.allFinite()) {
                    glVertex3f(y_end.x(), y_end.y(), y_end.z());
                }
                glEnd();

                // Z axis - blue (camera forward direction)
                glColor3f(0.0f, 0.0f, 1.0f);
                glLineWidth(4.0f);
                glBegin(GL_LINES);
                glVertex3f(current_pos.x(), current_pos.y(), current_pos.z());
                Eigen::Vector3d z_end = current_pos + current_rot.col(2) * frame_size;
                if (z_end.allFinite()) {
                    glVertex3f(z_end.x(), z_end.y(), z_end.z());
                }
                glEnd();
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error drawing trajectory: " << e.what() << std::endl;
    }
}

void Visualizer::drawFeaturePoints3D() {
    std::lock_guard<std::mutex> lock(points_mutex_);

    if (feature_points_3d_.empty())
        return;

    glPointSize(2.0f);
    glColor3f(0.0f, 0.8f, 1.0f);  // Cyan
    glBegin(GL_POINTS);
    for (const auto& pt : feature_points_3d_) {
        if (pt.allFinite())
            glVertex3f(pt.x(), pt.y(), pt.z());
    }
    glEnd();
}

void Visualizer::updateCameraPose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation, double timestamp) {
    std::lock_guard<std::mutex> lock(pose_mutex_);

    // Validate input data
    if (!position.allFinite() || !rotation.allFinite()) {
        std::cerr << "Visualizer: Invalid pose data received!" << std::endl;
        return;
    }

    camera_poses_.push_back(position);
    camera_rotations_.push_back(rotation);
    trajectory_timestamps_.push_back(timestamp);

    // Log progress occasionally
    static int log_counter = 0;
    if (++log_counter % 10 == 0) {
        std::cout << "Visualizer received pose " << camera_poses_.size() << ": [" << position.x() << ", "
                  << position.y() << ", " << position.z() << "] timestamp: " << timestamp << std::endl;
    }
}

void Visualizer::updateFeaturePoints3D(const std::vector<Eigen::Vector3d>& points) {
    std::lock_guard<std::mutex> lock(points_mutex_);

    // Filter valid points
    std::vector<Eigen::Vector3d> valid_points;
    for (const auto& pt : points) {
        if (pt.allFinite() && !std::isnan(pt.x()) && !std::isnan(pt.y()) && !std::isnan(pt.z())) {
            valid_points.push_back(pt);
        }
    }

    feature_points_3d_ = valid_points;

    // Debug info (occasionally)
    static int debug_counter = 0;
    if (++debug_counter % 100 == 0) {
        std::cout << "Updated 3D points: " << valid_points.size() << " valid out of " << points.size() << std::endl;
    }
}

void Visualizer::testWithSimpleData() {
    std::cout << "=== Testing with simple trajectory ===" << std::endl;

    // Generate simple circular trajectory
    for (int i = 0; i < 50; i++) {
        double angle = i * 0.1;
        Eigen::Vector3d pos(cos(angle), sin(angle), 0.1 * i);
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
        double timestamp = i * 0.1;

        updateCameraPose(pos, rot, timestamp);

        // Simple rendering test
        if (i % 10 == 0) {
            std::cout << "Test pose " << i << ": [" << pos.x() << ", " << pos.y() << ", " << pos.z() << "]"
                      << std::endl;
        }
    }
}

void Visualizer::waitUntilRenderReady() {
    std::unique_lock<std::mutex> lk(render_ready_mutex_);
    render_ready_cv_.wait(lk, [this] { return render_ready_; });
}

void Visualizer::drawCameraFrustum(const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot, double size) {
    if (!pos.allFinite() || !rot.allFinite())
        return;

    // Camera intrinsic parameters from config
    double fx = g_config.camera.fx;
    double fy = g_config.camera.fy;
    double cx = g_config.camera.cx;
    double cy = g_config.camera.cy;
    double width = g_config.camera.col;
    double height = g_config.camera.row;

    // Frustum depth
    double near_plane = size * 0.1;
    double far_plane = size;

    // Calculate corner rays in camera frame
    Eigen::Vector3d corners[4];
    corners[0] = Eigen::Vector3d((0 - cx) / fx, (0 - cy) / fy, 1.0);           // top-left
    corners[1] = Eigen::Vector3d((width - cx) / fx, (0 - cy) / fy, 1.0);       // top-right
    corners[2] = Eigen::Vector3d((width - cx) / fx, (height - cy) / fy, 1.0);  // bottom-right
    corners[3] = Eigen::Vector3d((0 - cx) / fx, (height - cy) / fy, 1.0);      // bottom-left

    // Transform to world coordinates and scale
    Eigen::Vector3d near_corners[4], far_corners[4];
    for (int i = 0; i < 4; i++) {
        corners[i].normalize();
        near_corners[i] = pos + rot * (corners[i] * near_plane);
        far_corners[i] = pos + rot * (corners[i] * far_plane);
    }

    // Draw frustum lines
    glColor3f(0.8f, 0.8f, 0.0f);  // Yellow color for frustum
    glLineWidth(1.5f);

    // Draw lines from camera center to far corners
    glBegin(GL_LINES);
    for (int i = 0; i < 4; i++) {
        glVertex3f(pos.x(), pos.y(), pos.z());
        glVertex3f(far_corners[i].x(), far_corners[i].y(), far_corners[i].z());
    }
    glEnd();

    // Draw near plane rectangle
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 4; i++) {
        glVertex3f(near_corners[i].x(), near_corners[i].y(), near_corners[i].z());
    }
    glEnd();

    // Draw far plane rectangle
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 4; i++) {
        glVertex3f(far_corners[i].x(), far_corners[i].y(), far_corners[i].z());
    }
    glEnd();
}

void Visualizer::adjustCameraViewToFitTrajectory() {
    if (camera_poses_.empty())
        return;

    std::lock_guard<std::mutex> lock(pose_mutex_);

    // Calculate trajectory bounds
    Eigen::Vector3d min_pos = camera_poses_[0];
    Eigen::Vector3d max_pos = camera_poses_[0];

    for (const auto& pose : camera_poses_) {
        if (pose.allFinite()) {
            min_pos = min_pos.cwiseMin(pose);
            max_pos = max_pos.cwiseMax(pose);
        }
    }

    // Calculate center and size of trajectory
    Eigen::Vector3d center = (min_pos + max_pos) / 2.0;
    Eigen::Vector3d size = max_pos - min_pos;
    double max_size = size.maxCoeff();

    // Add some padding
    double padding = std::max(max_size * 0.2, 2.0);  // 20% padding or minimum 2 units

    // Calculate camera position to view the entire trajectory
    double distance = max_size + padding;
    Eigen::Vector3d camera_pos = center + Eigen::Vector3d(0, -distance, distance * 0.5);

    // Update camera view if g_s_cam_ exists
    if (g_s_cam_) {
        g_s_cam_->SetModelViewMatrix(pangolin::ModelViewLookAt(camera_pos.x(), camera_pos.y(), camera_pos.z(),
                                                               center.x(), center.y(), center.z(), pangolin::AxisZ));
    }
}

}  // namespace utility