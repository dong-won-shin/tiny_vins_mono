#ifndef FAILURE_DETECTOR_H
#define FAILURE_DETECTOR_H

#include <iostream>
#include <Eigen/Dense>
#include "backend/sliding_window.h"
#include "frontend/feature_manager.h"

class FailureDetector {
public:
    FailureDetector(backend::SlidingWindow* sliding_window, FeatureManager* feature_manager);
    
    // Main failure detection method
    bool detectFailure(const Vector3d& last_P_end, const Matrix3d& last_R_end);
    
    // Individual failure detection methods
    bool detectFeatureFailure();
    bool detectIMUAccBiasFailure();
    bool detectIMUGyrBiasFailure();
    bool detectTranslationFailure(const Vector3d& last_P_end);
    bool detectZTranslationFailure(const Vector3d& last_P_end);
    bool detectRotationFailure(const Matrix3d& last_R_end);
    
    // Threshold setters
    void setFeatureThreshold(int threshold) { feature_threshold_ = threshold; }
    void setAccBiasThreshold(double threshold) { acc_bias_threshold_ = threshold; }
    void setGyrBiasThreshold(double threshold) { gyr_bias_threshold_ = threshold; }
    void setTranslationThreshold(double threshold) { translation_threshold_ = threshold; }
    void setZTranslationThreshold(double threshold) { z_translation_threshold_ = threshold; }
    void setRotationThreshold(double threshold) { rotation_threshold_ = threshold; }

private:
    // Member variables
    backend::SlidingWindow* sliding_window_;
    FeatureManager* feature_manager_;
    
    // Threshold parameters
    int feature_threshold_;
    double acc_bias_threshold_;
    double gyr_bias_threshold_;
    double translation_threshold_;
    double z_translation_threshold_;
    double rotation_threshold_;
};

#endif // FAILURE_DETECTOR_H 