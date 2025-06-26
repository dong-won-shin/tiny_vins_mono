#include "frontend/initialization/initializer.h"
#include "frontend/initialization/initial_alignment.h"
#include "frontend/feature_manager.h"

namespace frontend {

Initializer::Initializer(backend::SlidingWindow* sliding_window, FeatureManager* feature_manager,
                         MotionEstimator* motion_estimator,
                         std::map<double, common::ImageFrame>* all_image_frame, int* frame_count,
                         common::MarginalizationFlag* marginalization_flag, Vector3d* g,
                         const Matrix3d* r_ic, const Vector3d* t_ic)
    : sliding_window_(sliding_window),
      feature_manager_(feature_manager),
      motion_estimator_(motion_estimator),
      all_image_frame_(all_image_frame),
      frame_count_(frame_count),
      marginalization_flag_(marginalization_flag),
      g_(g),
      r_ic_(r_ic),
      t_ic_(t_ic) {}

bool Initializer::initialize() {
  // Check IMU excitation for initialization readiness
  if (!checkIMUExcitation(0.25)) {
    return false;
  }

  // Perform global SfM reconstruction
  if (!solveGlobalSfM()) {
    std::cout << "Global SfM reconstruction failed!" << std::endl;
    return false;
  }

  if (visualInitialAlign()) {
    std::cout << "visualInertialAlign() success!" << std::endl;
    return true;
  } else {
    std::cout << "misalign visual structure with IMU" << std::endl;
    return false;
  }
}

bool Initializer::checkIMUExcitation(double threshold) {
  if (all_image_frame_->size() <= 1) {
    std::cout << "Not enough image frames for IMU excitation check!" << std::endl;
    return false;
  }

  // Calculate average gravity vector from all frames
  Vector3d sum_g = Vector3d::Zero();
  int valid_frames = 0;

  for (auto frame_it = all_image_frame_->begin(); frame_it != all_image_frame_->end(); frame_it++) {
    if (frame_it == all_image_frame_->begin()) continue;  // Skip first frame

    if (frame_it->second.pre_integration != nullptr) {
      double dt = frame_it->second.pre_integration->sum_dt;
      if (dt > 0) {
        Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        sum_g += tmp_g;
        valid_frames++;
      }
    }
  }

  if (valid_frames == 0) {
    std::cout << "No valid frames with pre-integration data!" << std::endl;
    return false;
  }

  Vector3d aver_g = sum_g / valid_frames;

  // Calculate variance of gravity vectors
  double variance = 0.0;
  int variance_count = 0;

  for (auto frame_it = all_image_frame_->begin(); frame_it != all_image_frame_->end(); frame_it++) {
    if (frame_it == all_image_frame_->begin()) continue;  // Skip first frame

    if (frame_it->second.pre_integration != nullptr) {
      double dt = frame_it->second.pre_integration->sum_dt;
      if (dt > 0) {
        Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        Vector3d diff = tmp_g - aver_g;
        variance += diff.transpose() * diff;
        variance_count++;
      }
    }
  }

  if (variance_count <= 1) {
    std::cout << "Not enough frames for variance calculation!" << std::endl;
    return false;
  }

  double std_deviation = sqrt(variance / (variance_count - 1));

  std::cout << "IMU excitation check: std_deviation = " << std_deviation
            << ", threshold = " << threshold << std::endl;

  if (std_deviation < threshold) {
    std::cout << "IMU excitation not enough! Please move the device with more "
                 "rotation."
              << std::endl;
    return false;
  } else {
    std::cout << "IMU excitation sufficient for initialization." << std::endl;
    return true;
  }
}

bool Initializer::solveGlobalSfM() {
  std::cout << "Starting global SfM reconstruction..." << std::endl;

  map<int, Vector3d> sfm_tracked_points;

  // Prepare SfM features from feature manager
  vector<SFMFeature> sfm_f;
  for (auto& it_per_id : feature_manager_->feature) {
    int imu_j = it_per_id.start_frame - 1;
    SFMFeature tmp_feature;
    tmp_feature.state = false;
    tmp_feature.id = it_per_id.feature_id;
    tmp_feature.position[0] = 0.0;
    tmp_feature.position[1] = 0.0;
    tmp_feature.position[2] = 0.0;
    tmp_feature.depth = 0.0;
    for (auto& it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      Vector3d pts_j = it_per_frame.point;
      tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
    }
    sfm_f.push_back(tmp_feature);
  }

  std::cout << "Prepared " << sfm_f.size() << " SfM features for reconstruction" << std::endl;

  // Find relative pose between frames
  Matrix3d relative_R;
  Vector3d relative_T;
  int index;
  if (!relativePose(relative_R, relative_T, index)) {
    std::cout << "Global SfM failed: Not enough features or parallax. Move device "
                 "around!"
              << std::endl;
    return false;
  }

  std::cout << "Found relative pose between frame " << index << " and latest frame" << std::endl;

  // Perform global SfM reconstruction
  Quaterniond Q[(*frame_count_) + 1];
  Vector3d T[(*frame_count_) + 1];

  GlobalSFM sfm;
  if (!sfm.construct((*frame_count_) + 1, Q, T, index, relative_R, relative_T, sfm_f,
                     sfm_tracked_points)) {
    std::cout << "Global SfM reconstruction failed!" << std::endl;
    *marginalization_flag_ = common::MarginalizationFlag::MARGIN_OLD_KEYFRAME;
    return false;
  }

  std::cout << "Global SfM successful! Reconstructed " << sfm_tracked_points.size() << " 3D points"
            << std::endl;

  // Solve PnP for all frames to get camera poses
  if (!solvePnPForAllFrames(Q, T, sfm_tracked_points)) {
    std::cout << "PnP solving failed for non-keyframes!" << std::endl;
    return false;
  }

  std::cout << "Global SfM completed" << std::endl;
  return true;
}

bool Initializer::relativePose(Matrix3d& relative_R, Vector3d& relative_T, int& index) {
  // find previous frame which contians enough correspondance and parallex with
  // newest frame
  for (int i = 0; i < WINDOW_SIZE; i++) {
    frontend::Correspondences corres;
    corres = feature_manager_->getCorresponding(i, WINDOW_SIZE);

    if (corres.size() > 20) {
      double sum_parallax = 0;
      double average_parallax;
      for (int j = 0; j < int(corres.size()); j++) {
        Vector2d pts_0(corres[j].first(0), corres[j].first(1));
        Vector2d pts_1(corres[j].second(0), corres[j].second(1));
        double parallax = (pts_0 - pts_1).norm();
        sum_parallax = sum_parallax + parallax;
      }

      average_parallax = 1.0 * sum_parallax / int(corres.size());
      if (motion_estimator_->solveRelativeRT(corres, relative_R, relative_T)) {
        index = i;
        std::cout << "average_parallax " << average_parallax * 460 << " choose index " << index
                  << " and newest frame to triangulate "
                     "the whole structure"
                  << std::endl;
        return true;
      }
    }
  }
  return false;
}

bool Initializer::solvePnPForAllFrames(const Quaterniond Q[], const Vector3d T[],
                                       const map<int, Vector3d>& sfm_tracked_points) {
  map<double, common::ImageFrame>::iterator frame_it;
  map<int, Vector3d>::const_iterator it;
  frame_it = all_image_frame_->begin();

  for (int i = 0; frame_it != all_image_frame_->end(); frame_it++) {
    // Handle keyframes (those with SfM solutions)
    if ((frame_it->first) == (*sliding_window_)[i].timestamp) {
      frame_it->second.is_key_frame = true;
      auto temp = Q[i].toRotationMatrix() * (*r_ic_).transpose();
      frame_it->second.R = temp;
      frame_it->second.T = T[i];
      i++;
      continue;
    }

    // Handle frame timing
    if ((frame_it->first) > (*sliding_window_)[i].timestamp) {
      i++;
    }

    // Solve PnP for non-keyframes
    Matrix3d R_initial = (Q[i].inverse()).toRotationMatrix();
    Vector3d P_initial = -R_initial * T[i];

    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);

    frame_it->second.is_key_frame = false;

    // Collect 3D-2D correspondences
    vector<cv::Point3f> pts_3_vector;
    vector<cv::Point2f> pts_2_vector;

    for (auto& id_pts : frame_it->second.points) {
      int feature_id = id_pts.first;
      for (auto& i_p : id_pts.second) {
        it = sfm_tracked_points.find(feature_id);
        if (it != sfm_tracked_points.end()) {
          Vector3d world_pts = it->second;
          cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
          pts_3_vector.push_back(pts_3);
          Vector2d img_pts = i_p.head<2>();
          cv::Point2f pts_2(img_pts(0), img_pts(1));
          pts_2_vector.push_back(pts_2);
        }
      }
    }

    // Check if we have enough points for PnP
    if (pts_3_vector.size() < 6) {
      std::cout << "Not enough 3D-2D correspondences (" << pts_3_vector.size()
                << ") for PnP at frame " << frame_it->first << std::endl;
      return false;
    }

    // Solve PnP
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
      std::cout << "PnP solving failed for frame " << frame_it->first << std::endl;
      return false;
    }

    // Convert result back to pose
    cv::Rodrigues(rvec, r);
    MatrixXd R_pnp, tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);
    R_pnp = tmp_R_pnp.transpose();
    MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    T_pnp = R_pnp * (-T_pnp);

    frame_it->second.R = R_pnp * (*r_ic_).transpose();
    frame_it->second.T = T_pnp;
  }

  std::cout << "PnP solving completed for all frames" << std::endl;
  return true;
}

bool Initializer::visualInitialAlign() {
  VectorXd x;

  // solve for gravity vector and scale, gyroscope bias
  bool result = VisualIMUAlignment(*all_image_frame_, *sliding_window_, *g_, x);
  if (!result) {
    std::cout << "solve gravity vector failed!" << std::endl;
    return false;
  }

  // change state
  for (int i = 0; i <= *frame_count_; i++) {
    Matrix3d Ri = (*all_image_frame_)[(*sliding_window_)[i].timestamp].R;
    Vector3d Pi = (*all_image_frame_)[(*sliding_window_)[i].timestamp].T;

    (*sliding_window_)[i].P = Pi;
    (*sliding_window_)[i].R = Ri;
    (*all_image_frame_)[(*sliding_window_)[i].timestamp].is_key_frame = true;
  }

  VectorXd dep = feature_manager_->getDepthVector();
  for (int i = 0; i < dep.size(); i++) dep[i] = -1;
  feature_manager_->clearDepth(dep);

  // triangulat on cam pose , no tic
  Vector3d T_IC_TMP;
  T_IC_TMP.setZero();
  feature_manager_->triangulate(*sliding_window_, T_IC_TMP, *r_ic_);

  double scale = (x.tail<1>())(0);

  for (int i = 0; i <= WINDOW_SIZE; i++) {
    (*sliding_window_)[i].pre_integration->repropagate(Vector3d::Zero(), (*sliding_window_)[i].Bg);
  }

  for (int i = *frame_count_; i >= 0; i--)
    (*sliding_window_)[i].P =
        scale * (*sliding_window_)[i].P - (*sliding_window_)[i].R * (*t_ic_) -
        (scale * (*sliding_window_).front().P - (*sliding_window_).front().R * (*t_ic_));

  int kv = -1;
  map<double, common::ImageFrame>::iterator frame_i;
  for (frame_i = all_image_frame_->begin(); frame_i != all_image_frame_->end(); frame_i++) {
    if (frame_i->second.is_key_frame) {
      kv++;
      (*sliding_window_)[kv].V = frame_i->second.R * x.segment<3>(kv * 3);
    }
  }

  for (auto& it_per_id : feature_manager_->feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2)) continue;
    it_per_id.estimated_depth *= scale;
  }

  Matrix3d R0 = Utility::g2R(*g_);
  double yaw = Utility::R2ypr(R0 * (*sliding_window_).front().R).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  *g_ = R0 * (*g_);

  Matrix3d rot_diff = R0;
  for (int i = 0; i <= *frame_count_; i++) {
    (*sliding_window_)[i].P = rot_diff * (*sliding_window_)[i].P;
    (*sliding_window_)[i].R = rot_diff * (*sliding_window_)[i].R;
    (*sliding_window_)[i].V = rot_diff * (*sliding_window_)[i].V;
  }

  return true;
}

}  // namespace frontend