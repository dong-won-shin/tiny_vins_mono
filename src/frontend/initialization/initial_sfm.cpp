#include "frontend/initialization/initial_sfm.h"

namespace frontend {
namespace initialization {

GlobalSFM::GlobalSFM() {}

void GlobalSFM::triangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0, Eigen::Matrix<double, 3, 4>& Pose1,
                                 Vector2d& point0, Vector2d& point1, Vector3d& point_3d) {
    Matrix4d design_matrix = Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Vector4d triangulated_point;
    triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

bool GlobalSFM::solveFrameByPnP(Matrix3d& R_initial, Vector3d& P_initial, int i, vector<SFMFeature>& sfm_f) {
    vector<cv::Point2f> pts_2_vector;
    vector<cv::Point3f> pts_3_vector;
    for (int j = 0; j < feature_num; j++) {
        if (sfm_f[j].state != true)
            continue;
        Vector2d point2d;
        for (int k = 0; k < (int)sfm_f[j].observation.size(); k++) {
            if (sfm_f[j].observation[k].first == i) {
                Vector2d img_pts = sfm_f[j].observation[k].second;
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);
                cv::Point3f pts_3(sfm_f[j].position[0], sfm_f[j].position[1], sfm_f[j].position[2]);
                pts_3_vector.push_back(pts_3);
                break;
            }
        }
    }
    if (int(pts_2_vector.size()) < 15) {
        printf("unstable features tracking, please slowly move you device!\n");
        if (int(pts_2_vector.size()) < 10)
            return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1);
    if (!pnp_succ) {
        return false;
    }
    cv::Rodrigues(rvec, r);
    // cout << "r " << endl << r << endl;
    MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    R_initial = R_pnp;
    P_initial = T_pnp;
    return true;
}

void GlobalSFM::triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4>& Pose0, int frame1,
                                     Eigen::Matrix<double, 3, 4>& Pose1, vector<SFMFeature>& sfm_f) {
    assert(frame0 != frame1);
    for (int j = 0; j < feature_num; j++) {
        if (sfm_f[j].state == true)
            continue;
        bool has_0 = false, has_1 = false;
        Vector2d point0;
        Vector2d point1;
        for (int k = 0; k < (int)sfm_f[j].observation.size(); k++) {
            if (sfm_f[j].observation[k].first == frame0) {
                point0 = sfm_f[j].observation[k].second;
                has_0 = true;
            }
            if (sfm_f[j].observation[k].first == frame1) {
                point1 = sfm_f[j].observation[k].second;
                has_1 = true;
            }
        }
        if (has_0 && has_1) {
            Vector3d point_3d;
            triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
            sfm_f[j].state = true;
            sfm_f[j].position[0] = point_3d(0);
            sfm_f[j].position[1] = point_3d(1);
            sfm_f[j].position[2] = point_3d(2);
        }
    }
}

bool GlobalSFM::construct(int frame_num, Quaterniond* q, Vector3d* T, int reference_frame_id, const Matrix3d relative_R,
                          const Vector3d relative_T, vector<SFMFeature>& sfm_f,
                          map<int, Vector3d>& sfm_tracked_points) {
    const auto latest_frame_id = frame_num - 1;

    feature_num = sfm_f.size();

    // set reference frame
    q[reference_frame_id].w() = 1;
    q[reference_frame_id].x() = 0;
    q[reference_frame_id].y() = 0;
    q[reference_frame_id].z() = 0;
    T[reference_frame_id].setZero();

    // set latest frame
    q[latest_frame_id] = q[reference_frame_id] * Quaterniond(relative_R);
    T[latest_frame_id] = relative_T;

    // rotate to cam frame
    Matrix3d c_Rotation[frame_num];
    Vector3d c_Translation[frame_num];
    Quaterniond c_Quat[frame_num];
    double c_rotation[frame_num][4];
    double c_translation[frame_num][3];
    Eigen::Matrix<double, 3, 4> Pose[frame_num];

    c_Quat[reference_frame_id] = q[reference_frame_id].inverse();
    c_Rotation[reference_frame_id] = c_Quat[reference_frame_id].toRotationMatrix();
    c_Translation[reference_frame_id] = -1 * (c_Rotation[reference_frame_id] * T[reference_frame_id]);
    Pose[reference_frame_id].block<3, 3>(0, 0) = c_Rotation[reference_frame_id];
    Pose[reference_frame_id].block<3, 1>(0, 3) = c_Translation[reference_frame_id];

    c_Quat[latest_frame_id] = q[latest_frame_id].inverse();
    c_Rotation[latest_frame_id] = c_Quat[latest_frame_id].toRotationMatrix();
    c_Translation[latest_frame_id] = -1 * (c_Rotation[latest_frame_id] * T[latest_frame_id]);
    Pose[latest_frame_id].block<3, 3>(0, 0) = c_Rotation[latest_frame_id];
    Pose[latest_frame_id].block<3, 1>(0, 3) = c_Translation[latest_frame_id];

    // 1: trangulate from reference frame to latest frame by fixing the latest frame
    // 2: solve pnp from reference frame to latest frame
    auto fixed_frame_id = latest_frame_id;
    for (int i = reference_frame_id; i < latest_frame_id; i++) {
        // solve pnp
        if (i > reference_frame_id) {
            Matrix3d R_initial = c_Rotation[i - 1];
            Vector3d P_initial = c_Translation[i - 1];
            if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
                return false;
            c_Rotation[i] = R_initial;
            c_Translation[i] = P_initial;
            c_Quat[i] = c_Rotation[i];
            Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
            Pose[i].block<3, 1>(0, 3) = c_Translation[i];
        }

        // triangulate point based on the solve pnp result
        triangulateTwoFrames(i, Pose[i], fixed_frame_id, Pose[fixed_frame_id], sfm_f);
    }
    // 3: triangulate inbetween frames by fixing the reference frame
    fixed_frame_id = reference_frame_id;
    for (int i = reference_frame_id + 1; i < latest_frame_id; i++)
        triangulateTwoFrames(fixed_frame_id, Pose[fixed_frame_id], i, Pose[i], sfm_f);

    // 4: solve pnp from reference frame to oldest frame
    // 5: triangulate from reference frame to oldest frame by fixing reference frame
    for (int i = reference_frame_id - 1; i >= 0; i--) {
        // solve pnp
        Matrix3d R_initial = c_Rotation[i + 1];
        Vector3d P_initial = c_Translation[i + 1];
        if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
            return false;
        c_Rotation[i] = R_initial;
        c_Translation[i] = P_initial;
        c_Quat[i] = c_Rotation[i];
        Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
        Pose[i].block<3, 1>(0, 3) = c_Translation[i];

        // triangulate
        triangulateTwoFrames(i, Pose[i], fixed_frame_id, Pose[fixed_frame_id], sfm_f);
    }

    // 5: triangulate all other points
    for (int j = 0; j < feature_num; j++) {
        if (sfm_f[j].state == true)
            continue;

        if ((int)sfm_f[j].observation.size() >= 2) {
            Vector2d point0, point1;

            int frame_0 = sfm_f[j].observation[0].first;
            point0 = sfm_f[j].observation[0].second;
            int frame_1 = sfm_f[j].observation.back().first;
            point1 = sfm_f[j].observation.back().second;

            Vector3d point_3d;
            triangulatePoint(Pose[frame_0], Pose[frame_1], point0, point1, point_3d);
            sfm_f[j].state = true;
            sfm_f[j].position[0] = point_3d(0);
            sfm_f[j].position[1] = point_3d(1);
            sfm_f[j].position[2] = point_3d(2);
        }
    }

    // full BA
    ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
    // cout << " begin full BA " << endl;
    for (int i = 0; i < frame_num; i++) {
        // double array for ceres
        c_translation[i][0] = c_Translation[i].x();
        c_translation[i][1] = c_Translation[i].y();
        c_translation[i][2] = c_Translation[i].z();
        c_rotation[i][0] = c_Quat[i].w();
        c_rotation[i][1] = c_Quat[i].x();
        c_rotation[i][2] = c_Quat[i].y();
        c_rotation[i][3] = c_Quat[i].z();
        problem.AddParameterBlock(c_rotation[i], 4, local_parameterization);
        problem.AddParameterBlock(c_translation[i], 3);
        if (i == reference_frame_id) {
            problem.SetParameterBlockConstant(c_rotation[i]);
        }
        if (i == reference_frame_id || i == latest_frame_id) {
            problem.SetParameterBlockConstant(c_translation[i]);
        }
    }

    for (int i = 0; i < feature_num; i++) {
        if (sfm_f[i].state != true)
            continue;
        for (int j = 0; j < int(sfm_f[i].observation.size()); j++) {
            int l = sfm_f[i].observation[j].first;
            ceres::CostFunction* cost_function =
                ReprojectionError3D::Create(sfm_f[i].observation[j].second.x(), sfm_f[i].observation[j].second.y());

            problem.AddResidualBlock(cost_function, NULL, c_rotation[l], c_translation[l], sfm_f[i].position);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.2;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << "\n";
    if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03) {
        // cout << "vision only BA converge" << endl;
    } else {
        // cout << "vision only BA not converge " << endl;
        return false;
    }
    for (int i = 0; i < frame_num; i++) {
        q[i].w() = c_rotation[i][0];
        q[i].x() = c_rotation[i][1];
        q[i].y() = c_rotation[i][2];
        q[i].z() = c_rotation[i][3];
        q[i] = q[i].inverse();
        // cout << "final  q" << " i " << i <<"  " <<q[i].w() << "  " <<
        // q[i].vec().transpose() << endl;
    }
    for (int i = 0; i < frame_num; i++) {
        T[i] = -1 * (q[i] * Vector3d(c_translation[i][0], c_translation[i][1], c_translation[i][2]));
        // cout << "final  t" << " i " << i <<"  " << T[i](0) <<"  "<< T[i](1) <<"
        // "<< T[i](2) << endl;
    }
    for (int i = 0; i < (int)sfm_f.size(); i++) {
        if (sfm_f[i].state)
            sfm_tracked_points[sfm_f[i].id] =
                Vector3d(sfm_f[i].position[0], sfm_f[i].position[1], sfm_f[i].position[2]);
    }
    return true;
}

}  // namespace initialization
}  // namespace frontend
