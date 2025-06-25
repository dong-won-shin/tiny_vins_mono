#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <iostream>

#include "backend/factor/imu_factor.h"
#include "backend/factor/integration_base.h"
#include "backend/factor/marginalization_factor.h"
#include "backend/factor/pose_local_parameterization.h"
#include "backend/factor/projection_factor.h"
#include "backend/sliding_window.h"
#include "common/common_types.h"
#include "frontend/feature_manager.h"
#include "utility/config.h"

namespace backend {

class Optimizer {
public:
    Optimizer(SlidingWindow* sliding_window, FeatureManager* feature_manager);
    ~Optimizer();

    // Main optimization interface
    void optimize(MarginalizationFlag marginalization_flag);

    // Setter for extrinsic parameters
    void setExtrinsicParameters(const Vector3d& t_ic, const Matrix3d& r_ic);

    // Getter for extrinsic parameters
    Vector3d getTic() const {
        return t_ic_;
    }
    Matrix3d getRic() const {
        return r_ic_;
    }

private:
    // Optimization setup
    ceres::LossFunction* setupOptimizationProblem(ceres::Problem& problem);
    void addMarginalizationFactor(ceres::Problem& problem);
    void addIMUFactors(ceres::Problem& problem);
    int addFeatureFactors(ceres::Problem& problem, ceres::LossFunction* loss_function);
    void solveCeresProblem(ceres::Problem& problem);

    // Marginalization methods
    void marginalizeOldKeyframe();
    void marginalizeNewGeneralFrame();
    void addFeatureFactorsForMarginalization(factor::MarginalizationInfo* marginalization_info);
    void addIMUFactorForMarginalization(factor::MarginalizationInfo* marginalization_info);
    void performMarginalizationForOldKeyframe(factor::MarginalizationInfo* marginalization_info);
    void performMarginalizationForNewGeneralFrame(factor::MarginalizationInfo* marginalization_info);
    void handleMarginalization(MarginalizationFlag marginalization_flag);

    // Parameter management
    void prepareOptimizationParameters();
    void applyOptimizationResults();

    // Member variables
    SlidingWindow* sliding_window_;
    FeatureManager* feature_manager_;

    // Extrinsic parameters
    Vector3d t_ic_;
    Matrix3d r_ic_;

    // Optimization parameters
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedAndBiases[WINDOW_SIZE + 1][SIZE_SPEEDANDBIAS];
    double para_Feature[NUM_OF_FEATURES][SIZE_FEATURE];
    double para_Ex_Pose[SIZE_POSE];

    // Marginalization info
    factor::MarginalizationInfo* last_marginalization_info_;
    std::vector<double*> last_marginalization_parameter_blocks_;
};

}  // namespace backend

#endif  // OPTIMIZER_H