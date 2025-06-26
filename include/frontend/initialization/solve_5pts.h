#ifndef SOLVE_5PTS_H
#define SOLVE_5PTS_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "frontend/feature_manager.h"

using namespace std;
using namespace Eigen;

namespace frontend {

class MotionEstimator {
public:
  bool solveRelativeRT(const frontend::Correspondences& corres, Matrix3d &R, Vector3d &T);

private:
  double testTriangulation(const vector<cv::Point2f> &l, const vector<cv::Point2f> &r,
                           cv::Mat_<double> R, cv::Mat_<double> t);
  void decomposeE(cv::Mat E, cv::Mat_<double> &R1, cv::Mat_<double> &R2, cv::Mat_<double> &t1,
                  cv::Mat_<double> &t2);
};

}  // namespace frontend

#endif  // SOLVE_5PTS_H
