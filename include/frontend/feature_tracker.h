#ifndef FRONTEND__FEATURE_TRACKER_H
#define FRONTEND__FEATURE_TRACKER_H

#include <execinfo.h>
#include <csignal>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>

#include "common/camera_models/CameraFactory.h"
#include "common/camera_models/PinholeCamera.h"
#include "utility/config.h"

using namespace std;
using namespace common;
using namespace Eigen;

namespace frontend {

bool inBorder(const cv::Point2f& pt);

void filterByStatus(vector<cv::Point2f>& v, vector<uchar> status);
void filterByStatus(vector<int>& v, vector<uchar> status);

class FeatureTracker {
public:
    FeatureTracker();

    void detectAndTrack(const cv::Mat& _img, double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string& calib_file);

    void rejectWithFundamentalMatrix();

    void undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, next_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, next_pts;
    vector<cv::Point2f> prev_undistorted_pts, cur_undistorted_pts;
    vector<cv::Point2f> pts_velocity;
    vector<int> ids;
    vector<int> track_cnt;
    std::map<int, cv::Point2f> cur_undistorted_pts_map;
    std::map<int, cv::Point2f> prev_undistorted_pts_map;
    common::camera_models::CameraPtr m_camera;
    double cur_time;
    double prev_time;

    static int n_id;
};

}  // namespace frontend
#endif  // FRONTEND__FEATURE_TRACKER_H