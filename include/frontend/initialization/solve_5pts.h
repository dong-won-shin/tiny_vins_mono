#ifndef SOLVE_5PTS_H
#define SOLVE_5PTS_H

#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>
// #include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

namespace frontend {

class MotionEstimator {
public:
  bool solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &R, Vector3d &T);

private:
  double testTriangulation(const vector<cv::Point2f> &l, const vector<cv::Point2f> &r,
                           cv::Mat_<double> R, cv::Mat_<double> t);
  void decomposeE(cv::Mat E, cv::Mat_<double> &R1, cv::Mat_<double> &R2, cv::Mat_<double> &t1,
                  cv::Mat_<double> &t2);
};

}  // namespace frontend

#endif  // SOLVE_5PTS_H
