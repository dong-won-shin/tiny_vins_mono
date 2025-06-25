#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <Eigen/Dense>
#include <map>

#include "backend/sliding_window.h"
#include "common/common_types.h"
#include "common/image_frame.h"
#include "frontend/feature_manager.h"
#include "frontend/initialization/initial_sfm.h"
#include "frontend/initialization/solve_5pts.h"

namespace frontend {

class Estimator;  // Forward declaration

class Initializer {
public:
  Initializer(backend::SlidingWindow* sliding_window, FeatureManager* feature_manager,
              MotionEstimator* motion_estimator, std::map<double, ImageFrame>* all_image_frame,
              int* frame_count, MarginalizationFlag* marginalization_flag, Vector3d* g,
              const Matrix3d* r_ic, const Vector3d* t_ic);

  // Main initialization method
  bool initialize();

private:
  bool checkIMUExcitation(double threshold);
  bool solveGlobalSfM();
  bool relativePose(Matrix3d& relative_R, Vector3d& relative_T, int& index);
  bool solvePnPForAllFrames(const Quaterniond Q[], const Vector3d T[],
                            const std::map<int, Vector3d>& sfm_tracked_points);
  bool visualInitialAlign();

  // Member pointers to Estimator's data
  backend::SlidingWindow* sliding_window_;
  FeatureManager* feature_manager_;
  MotionEstimator* motion_estimator_;
  std::map<double, ImageFrame>* all_image_frame_;
  int* frame_count_;
  MarginalizationFlag* marginalization_flag_;
  Vector3d* g_;
  const Matrix3d* r_ic_;
  const Vector3d* t_ic_;
};

}  // namespace frontend

#endif  // INITIALIZER_H