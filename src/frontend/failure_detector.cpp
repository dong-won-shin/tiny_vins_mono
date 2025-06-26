#include "frontend/failure_detector.h"

namespace frontend {

FailureDetector::FailureDetector(backend::SlidingWindow* sliding_window, FeatureManager* feature_manager)
    : sliding_window_(sliding_window),
      feature_manager_(feature_manager),
      feature_threshold_(2),
      acc_bias_threshold_(2.5),
      gyr_bias_threshold_(1.0),
      translation_threshold_(5.0),
      z_translation_threshold_(1.0),
      rotation_threshold_(50.0) {}

bool FailureDetector::detectFailure(const Vector3d& last_P_end, const Matrix3d& last_R_end) {
    // Check feature failure
    if (detectFeatureFailure()) {
        return true;
    }

    // Check IMU bias failures
    if (detectIMUAccBiasFailure()) {
        return true;
    }

    if (detectIMUGyrBiasFailure()) {
        return true;
    }

    // Check translation failures
    if (detectTranslationFailure(last_P_end)) {
        return true;
    }

    if (detectZTranslationFailure(last_P_end)) {
        return true;
    }

    // Check rotation failure
    if (detectRotationFailure(last_R_end)) {
        return true;
    }

    return false;
}

bool FailureDetector::detectFeatureFailure() {
    if (feature_manager_->last_track_num_ < feature_threshold_) {
        std::cout << " little feature " << feature_manager_->last_track_num_ << std::endl;
        // return true; // Currently commented out in original code
    }
    return false;
}

bool FailureDetector::detectIMUAccBiasFailure() {
    if (sliding_window_->back().Ba.norm() > acc_bias_threshold_) {
        std::cout << " big IMU acc bias estimation " << sliding_window_->back().Ba.norm() << std::endl;
        return true;
    }
    return false;
}

bool FailureDetector::detectIMUGyrBiasFailure() {
    if (sliding_window_->back().Bg.norm() > gyr_bias_threshold_) {
        std::cout << " big IMU gyr bias estimation " << sliding_window_->back().Bg.norm() << std::endl;
        return true;
    }
    return false;
}

bool FailureDetector::detectTranslationFailure(const Vector3d& last_P_end) {
    Vector3d tmp_P = sliding_window_->back().P;
    if ((tmp_P - last_P_end).norm() > translation_threshold_) {
        std::cout << " big translation" << std::endl;
        return true;
    }
    return false;
}

bool FailureDetector::detectZTranslationFailure(const Vector3d& last_P_end) {
    Vector3d tmp_P = sliding_window_->back().P;
    if (abs(tmp_P.z() - last_P_end.z()) > z_translation_threshold_) {
        std::cout << " big z translation" << std::endl;
        return true;
    }
    return false;
}

bool FailureDetector::detectRotationFailure(const Matrix3d& last_R_end) {
    Matrix3d tmp_R = sliding_window_->back().R;
    Matrix3d delta_R = tmp_R.transpose() * last_R_end;
    Quaterniond delta_Q(delta_R);
    double delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;

    if (delta_angle > rotation_threshold_) {
        std::cout << " big delta_angle " << std::endl;
        // return true; // Currently commented out in original code
    }
    return false;
}

}  // namespace frontend