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
      status_(nullptr),
      view_top_down_(nullptr),
      view_follow_camera_(nullptr),
      view_mode_(VIEW_FREE) {}

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
    if (view_top_down_)
        delete view_top_down_;
    if (view_follow_camera_)
        delete view_follow_camera_;
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
                                       0.1, 10000),                            // near, far planes - increased far plane
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
        view_top_down_ = new pangolin::Var<bool>("ui.Top Down View", true, true);
        view_follow_camera_ = new pangolin::Var<bool>("ui.Follow Camera", false, true);

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

            // Clear screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            
            // Handle view mode changes through UI checkboxes
            static bool last_top_down = false;
            static bool last_follow_camera = false;
            
            if (view_top_down_ && *view_top_down_ && !last_top_down) {
                view_mode_ = VIEW_TOP_DOWN;
                if (view_follow_camera_) *view_follow_camera_ = false;
                std::cout << "Switched to Top-Down View" << std::endl;
            } else if (view_follow_camera_ && *view_follow_camera_ && !last_follow_camera) {
                view_mode_ = VIEW_FOLLOW_CAMERA;
                if (view_top_down_) *view_top_down_ = false;
                std::cout << "Switched to Follow Camera View" << std::endl;
            } else if (view_top_down_ && !*view_top_down_ && view_follow_camera_ && !*view_follow_camera_) {
                if (view_mode_ != VIEW_FREE) {
                    view_mode_ = VIEW_FREE;
                    std::cout << "Switched to Free View" << std::endl;
                }
            }
            
            if (view_top_down_) last_top_down = *view_top_down_;
            if (view_follow_camera_) last_follow_camera = *view_follow_camera_;
            
            // Update camera view based on mode
            updateCameraView();

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
                drawIMUTrajectory();
            }

            // Draw 3D map points if enabled
            if (show_points_ && *show_points_) {
                drawFeaturePoints3D();
            }

            // Update status
            if (status_) {
                std::stringstream ss;
                ss << "Points: " << feature_points_3d_.size();
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
        
        // Draw connection lines between camera and IMU (every 10th frame)
        if (camera_poses_.size() == imu_poses_.size()) {
            glColor3f(0.5f, 0.5f, 0.5f);  // Gray color for connections
            glLineWidth(1.0f);
            glBegin(GL_LINES);
            for (size_t i = 0; i < camera_poses_.size(); i += 10) {
                if (camera_poses_[i].allFinite() && imu_poses_[i].allFinite()) {
                    glVertex3f(camera_poses_[i].x(), camera_poses_[i].y(), camera_poses_[i].z());
                    glVertex3f(imu_poses_[i].x(), imu_poses_[i].y(), imu_poses_[i].z());
                }
            }
            glEnd();
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

void Visualizer::drawIMUTrajectory() {
    std::lock_guard<std::mutex> lock(pose_mutex_);

    if (imu_poses_.empty()) {
        return;
    }

    // Draw IMU trajectory
    glColor3f(1.0f, 0.5f, 0.0f);  // Orange color for IMU trajectory
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (const auto& pose : imu_poses_) {
        if (pose.allFinite()) {
            glVertex3f(pose.x(), pose.y(), pose.z());
        }
    }
    glEnd();

    // Draw IMU positions
    glPointSize(3.0f);
    glColor3f(1.0f, 0.3f, 0.0f);  // Darker orange for IMU positions
    glBegin(GL_POINTS);
    for (const auto& pose : imu_poses_) {
        if (pose.allFinite()) {
            glVertex3f(pose.x(), pose.y(), pose.z());
        }
    }
    glEnd();

    // Draw current IMU frame
    if (!imu_poses_.empty() && !imu_rotations_.empty()) {
        Eigen::Vector3d current_pos = imu_poses_.back();
        Eigen::Matrix3d current_rot = imu_rotations_.back();

        if (current_pos.allFinite() && current_rot.allFinite()) {
            // Draw current IMU frame
            drawIMUFrame(current_pos, current_rot, 0.12);
        }
    }
    
    // Draw connection line between camera and IMU
    if (!camera_poses_.empty() && !imu_poses_.empty()) {
        Eigen::Vector3d cam_pos = camera_poses_.back();
        Eigen::Vector3d imu_pos = imu_poses_.back();
        
        if (cam_pos.allFinite() && imu_pos.allFinite()) {
            glColor3f(0.7f, 0.7f, 0.7f);  // Gray color for connection
            glLineWidth(2.0f);
            glBegin(GL_LINES);
            glVertex3f(cam_pos.x(), cam_pos.y(), cam_pos.z());
            glVertex3f(imu_pos.x(), imu_pos.y(), imu_pos.z());
            glEnd();
        }
    }
}

void Visualizer::drawIMUFrame(const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot, double size) {
    if (!pos.allFinite() || !rot.allFinite())
        return;

    // IMU frame axes with distinct colors
    double frame_size = size;

    // X axis - red
    glColor3f(1.0f, 0.2f, 0.2f);
    glLineWidth(4.0f);
    glBegin(GL_LINES);
    glVertex3f(pos.x(), pos.y(), pos.z());
    Eigen::Vector3d x_end = pos + rot.col(0) * frame_size;
    if (x_end.allFinite()) {
        glVertex3f(x_end.x(), x_end.y(), x_end.z());
    }
    glEnd();

    // Y axis - green
    glColor3f(0.2f, 1.0f, 0.2f);
    glBegin(GL_LINES);
    glVertex3f(pos.x(), pos.y(), pos.z());
    Eigen::Vector3d y_end = pos + rot.col(1) * frame_size;
    if (y_end.allFinite()) {
        glVertex3f(y_end.x(), y_end.y(), y_end.z());
    }
    glEnd();

    // Z axis - blue
    glColor3f(0.2f, 0.2f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(pos.x(), pos.y(), pos.z());
    Eigen::Vector3d z_end = pos + rot.col(2) * frame_size;
    if (z_end.allFinite()) {
        glVertex3f(z_end.x(), z_end.y(), z_end.z());
    }
    glEnd();

    // Draw IMU box to make it more visible
    glColor3f(1.0f, 0.5f, 0.0f);  // Orange for IMU
    glLineWidth(2.0f);
    
    double box_size = size * 0.3;
    // Draw a small box around the IMU position
    glBegin(GL_LINE_LOOP);
    glVertex3f(pos.x() - box_size/2, pos.y() - box_size/2, pos.z() - box_size/2);
    glVertex3f(pos.x() + box_size/2, pos.y() - box_size/2, pos.z() - box_size/2);
    glVertex3f(pos.x() + box_size/2, pos.y() + box_size/2, pos.z() - box_size/2);
    glVertex3f(pos.x() - box_size/2, pos.y() + box_size/2, pos.z() - box_size/2);
    glEnd();
    
    glBegin(GL_LINE_LOOP);
    glVertex3f(pos.x() - box_size/2, pos.y() - box_size/2, pos.z() + box_size/2);
    glVertex3f(pos.x() + box_size/2, pos.y() - box_size/2, pos.z() + box_size/2);
    glVertex3f(pos.x() + box_size/2, pos.y() + box_size/2, pos.z() + box_size/2);
    glVertex3f(pos.x() - box_size/2, pos.y() + box_size/2, pos.z() + box_size/2);
    glEnd();
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
}

void Visualizer::updateIMUPose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation, double timestamp) {
    std::lock_guard<std::mutex> lock(pose_mutex_);

    // Validate input data
    if (!position.allFinite() || !rotation.allFinite()) {
        std::cerr << "Visualizer: Invalid IMU pose data received!" << std::endl;
        return;
    }

    imu_poses_.push_back(position);
    imu_rotations_.push_back(rotation);
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

void Visualizer::updateCameraView() {
    if (!g_s_cam_ || camera_poses_.empty()) return;
    
    std::lock_guard<std::mutex> lock(pose_mutex_);
    
    switch (view_mode_) {
        case VIEW_TOP_DOWN: {
            // Calculate bounds of trajectory
            Eigen::Vector3d min_pos = camera_poses_[0];
            Eigen::Vector3d max_pos = camera_poses_[0];
            
            for (const auto& pos : camera_poses_) {
                if (pos.allFinite()) {
                    min_pos = min_pos.cwiseMin(pos);
                    max_pos = max_pos.cwiseMax(pos);
                }
            }
            
            // Include IMU poses if available
            for (const auto& pos : imu_poses_) {
                if (pos.allFinite()) {
                    min_pos = min_pos.cwiseMin(pos);
                    max_pos = max_pos.cwiseMax(pos);
                }
            }
            
            // Calculate center and size
            Eigen::Vector3d center = (min_pos + max_pos) / 2.0;
            Eigen::Vector3d size = max_pos - min_pos;
            
            // Calculate appropriate height based on trajectory size
            double max_dimension = std::max(size.x(), size.y());
            double height = max_dimension * 1.2; // 20% margin
            if (height < 10.0) height = 10.0; // Minimum height
            
            // Set camera to look down from above
            g_s_cam_->SetModelViewMatrix(
                pangolin::ModelViewLookAt(
                    center.x(), center.y(), center.z() + height,  // Camera position (above)
                    center.x(), center.y(), center.z(),           // Look at center
                    1.0, 0.0, 0.0                                 // Up vector (X-axis)
                )
            );
            break;
        }
        
        case VIEW_FOLLOW_CAMERA: {
            if (!camera_poses_.empty() && !camera_rotations_.empty()) {
                Eigen::Vector3d cam_pos = camera_poses_.back();
                Eigen::Matrix3d cam_rot = camera_rotations_.back();
                
                // Position behind and above the camera
                Eigen::Vector3d offset = cam_rot * Eigen::Vector3d(0, 0, -3.0); // 3m behind
                offset += Eigen::Vector3d(0, 0, 1.0); // 1m up
                Eigen::Vector3d view_pos = cam_pos + offset;
                
                // Look at a point in front of the camera
                Eigen::Vector3d look_at = cam_pos + cam_rot * Eigen::Vector3d(0, 0, 2.0);
                
                g_s_cam_->SetModelViewMatrix(
                    pangolin::ModelViewLookAt(
                        view_pos.x(), view_pos.y(), view_pos.z(),
                        look_at.x(), look_at.y(), look_at.z(),
                        0.0, 0.0, 1.0  // Up vector (Z-axis)
                    )
                );
            }
            break;
        }
        
        case VIEW_FREE:
        default:
            // Don't modify camera in free mode
            break;
    }
}

}  // namespace utility