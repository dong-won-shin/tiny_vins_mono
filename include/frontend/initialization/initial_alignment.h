#ifndef INITIAL_ALIGNMENT_H
#define INITIAL_ALIGNMENT_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>

#include "backend/factor/imu_factor.h"
#include "backend/sliding_window.h"
#include "common/image_frame.h"
#include "frontend/feature_manager.h"
#include "utility/utility.h"

using namespace Eigen;
using namespace std;

namespace frontend {
namespace initialization {

bool VisualIMUAlignment(map<double, common::ImageFrame> const& all_image_frame, backend::SlidingWindow& sliding_window,
                        Vector3d& g, VectorXd& x);

}  // namespace initialization
}  // namespace frontend

#endif  // INITIAL_ALIGNMENT_H