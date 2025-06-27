#ifndef FRONTEND__FEATURE_MANAGER_H
#define FRONTEND__FEATURE_MANAGER_H

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <list>
#include <numeric>
#include <vector>

#include "backend/sliding_window.h"
#include "common/image_frame.h"
#include "utility/config.h"

using namespace std;
using namespace Eigen;

namespace frontend {

using Correspondence = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
using Correspondences = std::vector<Correspondence>;

class FeaturePerFrame {
public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1>& _point) {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5);
        velocity.y() = _point(6);
    }
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};

class FeaturePerId {
public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag;  // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame), used_num(0), estimated_depth(-1.0), solve_flag(0) {}

    int endFrame();
};

class FeatureManager {
public:
    FeatureManager();

    void clearState();

    int getFeatureCount();

    bool addFeatureAndCheckParallax(int frame_count, const common::ImageData& image);
    Correspondences getCorresponding(int frame_count_l, int frame_count_r);

    // void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd& x);
    void removeFailures();
    void clearDepth(const VectorXd& x);
    VectorXd getDepthVector();
    void triangulate(const backend::SlidingWindow& sliding_window, const Vector3d& t_ic, const Matrix3d& r_ic);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R,
                              Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier();
    list<FeaturePerId> feature_bank_;
    int last_track_num_;

private:
    double compensatedParallax2(const FeaturePerId& it_per_id, int frame_count);
};

}  // namespace frontend

#endif  // FRONTEND__FEATURE_MANAGER_H