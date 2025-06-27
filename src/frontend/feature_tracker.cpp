#include "frontend/feature_tracker.h"

using namespace utility;

namespace frontend {

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f& pt) {
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < g_config.camera.col - BORDER_SIZE && BORDER_SIZE <= img_y &&
           img_y < g_config.camera.row - BORDER_SIZE;
}

void filterByStatus(vector<cv::Point2f>& v, vector<uchar> status) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void filterByStatus(vector<int>& v, vector<uchar> status) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker() {}

void FeatureTracker::setMask() {
    if (g_config.feature_tracker.fisheye)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(g_config.camera.row, g_config.camera.col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < next_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(next_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(),
         [](const pair<int, pair<cv::Point2f, int>>& a, const pair<int, pair<cv::Point2f, int>>& b) {
             return a.first > b.first;
         });

    next_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto& it : cnt_pts_id) {
        if (mask.at<uchar>(it.second.first) == 255) {
            next_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, g_config.feature_tracker.min_dist, 0, -1);
        }
    }
}

void FeatureTracker::addPoints() {
    for (auto& p : n_pts) {
        next_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::detectAndTrack(const cv::Mat& _img, double _cur_time) {
    cv::Mat img;
    cur_time = _cur_time;

    // equalize histogram
    if (g_config.feature_tracker.equalize) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(_img, img);
    } else
        img = _img;

    if (next_img.empty()) {
        prev_img = cur_img = next_img = img;
    } else {
        next_img = img;
    }

    next_pts.clear();

    if (cur_pts.size() > 0) {
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, next_img, cur_pts, next_pts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(next_pts.size()); i++)
            if (status[i] && !inBorder(next_pts[i]))
                status[i] = 0;

        filterByStatus(prev_pts, status);
        filterByStatus(cur_pts, status);
        filterByStatus(next_pts, status);
        filterByStatus(ids, status);
        filterByStatus(cur_undistorted_pts, status);
        filterByStatus(track_cnt, status);
    }

    for (auto& n : track_cnt)
        n++;

    rejectWithFundamentalMatrix();
    setMask();

    int supplementary_points_count = g_config.feature_tracker.max_cnt - static_cast<int>(next_pts.size());
    if (supplementary_points_count > 0) {
        if (mask.empty())
            cout << "mask is empty " << endl;
        if (mask.type() != CV_8UC1)
            cout << "mask type wrong " << endl;
        if (mask.size() != next_img.size())
            cout << "wrong size " << endl;

        cv::goodFeaturesToTrack(next_img, n_pts, supplementary_points_count, 0.01, g_config.feature_tracker.min_dist,
                                mask);
    } else
        n_pts.clear();

    addPoints();

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_undistorted_pts = cur_undistorted_pts;
    cur_img = next_img;
    cur_pts = next_pts;
    undistortedPoints();
    prev_time = cur_time;
}

void FeatureTracker::rejectWithFundamentalMatrix() {
    if (next_pts.size() >= 8) {
        vector<cv::Point2f> undistorted_cur_pts(cur_pts.size()), undistorted_next_pts(next_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++) {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = g_config.camera.focal_length * tmp_p.x() / tmp_p.z() + g_config.camera.col / 2.0;
            tmp_p.y() = g_config.camera.focal_length * tmp_p.y() / tmp_p.z() + g_config.camera.row / 2.0;
            undistorted_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(next_pts[i].x, next_pts[i].y), tmp_p);
            tmp_p.x() = g_config.camera.focal_length * tmp_p.x() / tmp_p.z() + g_config.camera.col / 2.0;
            tmp_p.y() = g_config.camera.focal_length * tmp_p.y() / tmp_p.z() + g_config.camera.row / 2.0;
            undistorted_next_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(undistorted_cur_pts, undistorted_next_pts, cv::FM_RANSAC,
                               g_config.feature_tracker.f_threshold, 0.99, status);
        int size_a = cur_pts.size();
        filterByStatus(prev_pts, status);
        filterByStatus(cur_pts, status);
        filterByStatus(next_pts, status);
        filterByStatus(cur_undistorted_pts, status);
        filterByStatus(ids, status);
        filterByStatus(track_cnt, status);
    }
}

bool FeatureTracker::updateID(unsigned int i) {
    if (i < ids.size()) {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    } else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string& calib_file) {
    m_camera = common::camera_models::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::undistortedPoints() {
    cur_undistorted_pts.clear();
    cur_undistorted_pts_map.clear();
    // cv::undistortPoints(cur_pts, undistorted_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++) {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_undistorted_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_undistorted_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        // printf("cur pts id %d %f %f", ids[i], cur_undistorted_pts[i].x, cur_undistorted_pts[i].y);
    }
    // caculate points velocity
    if (!prev_undistorted_pts_map.empty()) {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_undistorted_pts.size(); i++) {
            if (ids[i] != -1) {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_undistorted_pts_map.find(ids[i]);
                if (it != prev_undistorted_pts_map.end()) {
                    double v_x = (cur_undistorted_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_undistorted_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                } else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            } else {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    } else {
        for (unsigned int i = 0; i < cur_pts.size(); i++) {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_undistorted_pts_map = cur_undistorted_pts_map;
}

}  // namespace frontend