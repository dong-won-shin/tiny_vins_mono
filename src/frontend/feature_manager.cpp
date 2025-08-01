#include "frontend/feature_manager.h"

namespace frontend {

int FeaturePerId::endFrame() {
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager() {}

void FeatureManager::clearState() {
    feature_bank_.clear();
}

int FeatureManager::getFeatureCount() {
    int cnt = 0;
    for (auto& it : feature_bank_) {
        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2) {
            cnt++;
        }
    }
    return cnt;
}

bool FeatureManager::addFeatureAndCheckParallax(int frame_count, const common::ImageData& image) {
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num_ = 0;

    // add features to feature bank
    for (auto& feature_id_and_info : image) {
        int feature_id = feature_id_and_info.first;
        FeaturePerFrame feature_per_frame(feature_id_and_info.second);

        auto it = find_if(feature_bank_.begin(), feature_bank_.end(),
                          [feature_id](const FeaturePerId& it) { return it.feature_id == feature_id; });

        if (it == feature_bank_.end()) {
            // new feature
            feature_bank_.push_back(FeaturePerId(feature_id, frame_count));
            feature_bank_.back().feature_per_frame.push_back(feature_per_frame);
        } else if (it->feature_id == feature_id) {
            // tracked feature
            it->feature_per_frame.push_back(feature_per_frame);
            last_track_num_++;
        }
    }

    // check parallax
    if (frame_count < 2 || last_track_num_ < 20) {
        // novel view
        return true;
    }

    for (auto& it_per_id : feature_bank_) {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1) {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0) {
        // novel view
        return true;
    } else {
        // enough parallax, new keyframe
        return (parallax_sum / parallax_num) >= (g_config.estimator.min_parallax / g_config.camera.focal_length);
    }
}

Correspondences FeatureManager::getCorresponding(int frame_count_l, int frame_count_r) {
    frontend::Correspondences corres;
    for (auto& it : feature_bank_) {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r) {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            // ray vector in frame_count_l of same feature_id
            a = it.feature_per_frame[idx_l].ray_vector;

            // ray vector in frame_count_r of same feature_id
            b = it.feature_per_frame[idx_r].ray_vector;

            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd& x) {
    int feature_index = -1;
    for (auto& it_per_id : feature_bank_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        if (it_per_id.estimated_depth < 0) {
            it_per_id.solve_flag = 2;
        } else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures() {
    for (auto it = feature_bank_.begin(), it_next = feature_bank_.begin(); it != feature_bank_.end(); it = it_next) {
        it_next++;
        if (it->solve_flag == 2)
            feature_bank_.erase(it);
    }
}

void FeatureManager::clearDepth(const VectorXd& x) {
    int feature_index = -1;
    for (auto& it_per_id : feature_bank_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

VectorXd FeatureManager::getDepthVector() {
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto& it_per_id : feature_bank_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
    }
    return dep_vec;
}

void FeatureManager::triangulateAcrossAllViews(const backend::SlidingWindow& sliding_window, const Vector3d& t_ic,
                                               const Matrix3d& r_ic) {
    for (auto& it_per_id : feature_bank_) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;

        int imu_i = it_per_id.start_frame;
        int imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = sliding_window[imu_i].P + sliding_window[imu_i].R * t_ic;
        Eigen::Matrix3d R0 = sliding_window[imu_i].R * r_ic;
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto& it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;

            Eigen::Vector3d t1 = sliding_window[imu_j].P + sliding_window[imu_j].R * t_ic;
            Eigen::Matrix3d R1 = sliding_window[imu_j].R * r_ic;
            Eigen::Vector3d relative_t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d relative_R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = relative_R.transpose();
            P.rightCols<1>() = -relative_R.transpose() * relative_t;
            Eigen::Vector3d f = it_per_frame.ray_vector.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        assert(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        // it_per_id->estimated_depth = -b / A;
        // it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        // it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1) {
            it_per_id.estimated_depth = g_config.estimator.init_depth;
        }
    }
}

void FeatureManager::removeOutlier() {
    assert(false);
    int i = -1;
    for (auto it = feature_bank_.begin(), it_next = feature_bank_.begin(); it != feature_bank_.end(); it = it_next) {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true) {
            feature_bank_.erase(it);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R,
                                          Eigen::Vector3d new_P) {
    for (auto it = feature_bank_.begin(), it_next = feature_bank_.begin(); it != feature_bank_.end(); it = it_next) {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].ray_vector;
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2) {
                feature_bank_.erase(it);
                continue;
            } else {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = g_config.estimator.init_depth;
            }
        }
    }
}

void FeatureManager::removeBack() {
    for (auto it = feature_bank_.begin(), it_next = feature_bank_.begin(); it != feature_bank_.end(); it = it_next) {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature_bank_.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count) {
    for (auto it = feature_bank_.begin(), it_next = feature_bank_.begin(); it != feature_bank_.end(); it = it_next) {
        it_next++;

        if (it->start_frame == frame_count) {
            it->start_frame--;
        } else {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature_bank_.erase(it);
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId& it_per_id, int frame_count) {
    // check the second last frame is keyframe or not
    // parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame& frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame& frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.ray_vector;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.ray_vector;
    Vector3d p_i_comp;

    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}

}  // namespace frontend