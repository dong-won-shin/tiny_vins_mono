#pragma once
#include "backend/factor/imu_factor.h"
#include "frontend/feature_manager.h"
#include "common/image_frame.h"
#include "utility/utility.h"
#include "backend/sliding_window.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>

using namespace Eigen;
using namespace std;

namespace initial_alignment {

bool VisualIMUAlignment(map<double, ImageFrame> const &all_image_frame, backend::SlidingWindow &sliding_window, Vector3d &g, VectorXd &x);
}