#include "backend/optimizer.h"

namespace backend {

Optimizer::Optimizer(SlidingWindow* sliding_window, FeatureManager* feature_manager) 
    : sliding_window_(sliding_window), feature_manager_(feature_manager), last_marginalization_info_(nullptr) {
    t_ic_ = Vector3d::Zero();
    r_ic_ = Matrix3d::Identity();
}

Optimizer::~Optimizer() {
    if (last_marginalization_info_ != nullptr) {
        delete last_marginalization_info_;
        last_marginalization_info_ = nullptr;
    }
}

void Optimizer::setExtrinsicParameters(const Vector3d& t_ic, const Matrix3d& r_ic) {
    t_ic_ = t_ic;
    r_ic_ = r_ic;
}

void Optimizer::optimize(MarginalizationFlag marginalization_flag) {
    // STEP 1: Setup optimization problem and parameter blocks
    ceres::Problem problem;
    ceres::LossFunction *loss_function = setupOptimizationProblem(problem);
    
    // STEP 2: Add various constraint factors
    addMarginalizationFactor(problem);
    addIMUFactors(problem);
    addFeatureFactors(problem, loss_function);
    
    // STEP 3: Solve the optimization problem
    solveCeresProblem(problem);
    
    // STEP 4: Apply results and handle marginalization
    applyOptimizationResults();
    handleMarginalization(marginalization_flag);
}

ceres::LossFunction* Optimizer::setupOptimizationProblem(ceres::Problem& problem) {
    // Setup loss function
    ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
    
    // Add pose and speed-bias parameter blocks for sliding window
    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization); // R, P
        problem.AddParameterBlock(para_SpeedAndBiases[i], SIZE_SPEEDANDBIAS); // V, Ba, Bg
    }

    // Add extrinsic parameter block
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_Ex_Pose, SIZE_POSE, local_parameterization);
    problem.SetParameterBlockConstant(para_Ex_Pose);

    // Prepare optimization parameters
    prepareOptimizationParameters();
    
    return loss_function;
}

void Optimizer::addMarginalizationFactor(ceres::Problem& problem) {
    if (last_marginalization_info_) {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
        problem.AddResidualBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks_);
    }
}

void Optimizer::addIMUFactors(ceres::Problem& problem) {
    for (int i = 0; i < WINDOW_SIZE; i++) {
        int j = i + 1;
        if ((*sliding_window_)[j].pre_integration->sum_dt > 10.0)
            continue;
        IMUFactor *imu_factor = new IMUFactor((*sliding_window_)[j].pre_integration);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedAndBiases[i], para_Pose[j],
                                 para_SpeedAndBiases[j]);
    }
}

int Optimizer::addFeatureFactors(ceres::Problem& problem, ceres::LossFunction* loss_function) {
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : feature_manager_->feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
            if (imu_i == imu_j) {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose,
                                        para_Feature[feature_index]);
            f_m_cnt++;
        }
    }
    return f_m_cnt;
}

void Optimizer::solveCeresProblem(ceres::Problem& problem) {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_solver_time_in_seconds = g_config.estimator.solver_time;
    options.max_num_iterations = g_config.estimator.num_iterations;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}

void Optimizer::applyOptimizationResults() {
    Vector3d origin_R0 = Utility::R2ypr((*sliding_window_).front().R);
    Vector3d origin_P0 = (*sliding_window_).front().P;
    
    Vector3d origin_R00 = Utility::R2ypr(
        Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4], para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    // TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
        std::cout << "euler singular point!" << std::endl;
        rot_diff = (*sliding_window_).front().R * Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4], para_Pose[0][5])
                               .toRotationMatrix()
                               .transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++) {
        (*sliding_window_)[i].R = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5])
                               .normalized()
                               .toRotationMatrix();

        (*sliding_window_)[i].P = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0], para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) +
                origin_P0;

        (*sliding_window_)[i].V = rot_diff * Vector3d(para_SpeedAndBiases[i][0], para_SpeedAndBiases[i][1], para_SpeedAndBiases[i][2]);

        (*sliding_window_)[i].Ba = Vector3d(para_SpeedAndBiases[i][3], para_SpeedAndBiases[i][4], para_SpeedAndBiases[i][5]);

        (*sliding_window_)[i].Bg = Vector3d(para_SpeedAndBiases[i][6], para_SpeedAndBiases[i][7], para_SpeedAndBiases[i][8]);
    }

    t_ic_ = Vector3d(para_Ex_Pose[0], para_Ex_Pose[1], para_Ex_Pose[2]);
    r_ic_ = Quaterniond(para_Ex_Pose[6], para_Ex_Pose[3], para_Ex_Pose[4], para_Ex_Pose[5]).toRotationMatrix();

    VectorXd dep = feature_manager_->getDepthVector();
    for (int i = 0; i < feature_manager_->getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    feature_manager_->setDepth(dep);
}

void Optimizer::handleMarginalization(MarginalizationFlag marginalization_flag) {
    if (marginalization_flag == MarginalizationFlag::MARGIN_OLD_KEYFRAME) {
        marginalizeOldKeyframe();
    } 
    else if (marginalization_flag == MarginalizationFlag::MARGIN_NEW_GENERAL_FRAME) {
        marginalizeNewGeneralFrame();
    }
}

void Optimizer::prepareOptimizationParameters() {
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        // P
        para_Pose[i][0] = (*sliding_window_)[i].P.x();
        para_Pose[i][1] = (*sliding_window_)[i].P.y();
        para_Pose[i][2] = (*sliding_window_)[i].P.z();

        // R
        Quaterniond q{(*sliding_window_)[i].R};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        // V
        para_SpeedAndBiases[i][0] = (*sliding_window_)[i].V.x();
        para_SpeedAndBiases[i][1] = (*sliding_window_)[i].V.y();
        para_SpeedAndBiases[i][2] = (*sliding_window_)[i].V.z();

        // Ba
        para_SpeedAndBiases[i][3] = (*sliding_window_)[i].Ba.x();
        para_SpeedAndBiases[i][4] = (*sliding_window_)[i].Ba.y();
        para_SpeedAndBiases[i][5] = (*sliding_window_)[i].Ba.z();

        // Bg
        para_SpeedAndBiases[i][6] = (*sliding_window_)[i].Bg.x();
        para_SpeedAndBiases[i][7] = (*sliding_window_)[i].Bg.y();
        para_SpeedAndBiases[i][8] = (*sliding_window_)[i].Bg.z();
    }

    // Camera-IMU Extrinsic
    para_Ex_Pose[0] = t_ic_.x(); // translation
    para_Ex_Pose[1] = t_ic_.y();
    para_Ex_Pose[2] = t_ic_.z();

    Quaterniond q{r_ic_}; // rotation
    para_Ex_Pose[3] = q.x();
    para_Ex_Pose[4] = q.y();
    para_Ex_Pose[5] = q.z();
    para_Ex_Pose[6] = q.w();

    // Feature point depth
    VectorXd dep = feature_manager_->getDepthVector();
    for (int i = 0; i < feature_manager_->getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
}

void Optimizer::marginalizeOldKeyframe() {
    MarginalizationInfo *marginalization_info = new MarginalizationInfo();
    prepareOptimizationParameters();

    if (last_marginalization_info_) {
        vector<int> drop_set;
        for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); i++) {
            bool does_last_margin_param_block_contain_the_old_keyframe_pose_and_speed_bias = 
                last_marginalization_parameter_blocks_[i] == para_Pose[0] ||
                last_marginalization_parameter_blocks_[i] == para_SpeedAndBiases[0];

            if (does_last_margin_param_block_contain_the_old_keyframe_pose_and_speed_bias)
                drop_set.push_back(i);
        }
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
        ResidualBlockInfo *residual_block_info =
            new ResidualBlockInfo(marginalization_factor, NULL, last_marginalization_parameter_blocks_, drop_set);
        marginalization_info->addResidualBlockInfo(residual_block_info);
    }

    addIMUFactorForMarginalization(marginalization_info);
    addFeatureFactorsForMarginalization(marginalization_info);

    performMarginalizationForOldKeyframe(marginalization_info);
}

void Optimizer::marginalizeNewGeneralFrame() {
    bool does_last_margin_param_block_contain_the_new_general_frame_pose = 
        std::count(std::begin(last_marginalization_parameter_blocks_),
                   std::end(last_marginalization_parameter_blocks_), 
                   para_Pose[WINDOW_SIZE - 1]);

    if (does_last_margin_param_block_contain_the_new_general_frame_pose) 
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        prepareOptimizationParameters();

        if (last_marginalization_info_) {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); i++) {
                assert(last_marginalization_parameter_blocks_[i] != para_SpeedAndBiases[WINDOW_SIZE - 1]);
                if (last_marginalization_parameter_blocks_[i] == para_Pose[WINDOW_SIZE - 1])
                    drop_set.push_back(i);
            }
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                marginalization_factor, NULL, last_marginalization_parameter_blocks_, drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        performMarginalizationForNewGeneralFrame(marginalization_info);
    }
}

void Optimizer::addIMUFactorForMarginalization(MarginalizationInfo* marginalization_info) {
    if ((*sliding_window_)[1].pre_integration->sum_dt < 10.0) {
        IMUFactor *imu_factor = new IMUFactor((*sliding_window_)[1].pre_integration);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            imu_factor, NULL,
            vector<double *>{para_Pose[0], para_SpeedAndBiases[0], para_Pose[1], para_SpeedAndBiases[1]},
            vector<int>{0, 1});
        marginalization_info->addResidualBlockInfo(residual_block_info);
    }
}

void Optimizer::addFeatureFactorsForMarginalization(MarginalizationInfo* marginalization_info) {
    ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
    int feature_index = -1;
    for (auto &it_per_id : feature_manager_->feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        if (imu_i != 0)
            continue;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
            if (imu_i == imu_j)
                continue;

            Vector3d pts_j = it_per_frame.point;
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
            ResidualBlockInfo *residual_block_info =
                new ResidualBlockInfo(f, loss_function,
                                      vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose,
                                                       para_Feature[feature_index]},
                                      vector<int>{0, 3});
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
    }
}

void Optimizer::performMarginalizationForOldKeyframe(MarginalizationInfo* marginalization_info) {
    marginalization_info->preMarginalize();
    marginalization_info->marginalize();

    std::unordered_map<long, double *> addr_shift;
    for (int i = 1; i <= WINDOW_SIZE; i++) {
        addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
        addr_shift[reinterpret_cast<long>(para_SpeedAndBiases[i])] = para_SpeedAndBiases[i - 1];
    }
    addr_shift[reinterpret_cast<long>(para_Ex_Pose)] = para_Ex_Pose;
    vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

    if (last_marginalization_info_)
        delete last_marginalization_info_;
    last_marginalization_info_ = marginalization_info;
    last_marginalization_parameter_blocks_ = parameter_blocks;
}

void Optimizer::performMarginalizationForNewGeneralFrame(MarginalizationInfo* marginalization_info) {
    marginalization_info->preMarginalize();
    marginalization_info->marginalize();

    std::unordered_map<long, double *> addr_shift;
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        if (i == WINDOW_SIZE - 1)
            continue;
        else if (i == WINDOW_SIZE) {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedAndBiases[i])] = para_SpeedAndBiases[i - 1];
        } else {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
            addr_shift[reinterpret_cast<long>(para_SpeedAndBiases[i])] = para_SpeedAndBiases[i];
        }
    }
    addr_shift[reinterpret_cast<long>(para_Ex_Pose)] = para_Ex_Pose;

    vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
    if (last_marginalization_info_)
        delete last_marginalization_info_;
    last_marginalization_info_ = marginalization_info;
    last_marginalization_parameter_blocks_ = parameter_blocks;   
}

} // namespace backend 