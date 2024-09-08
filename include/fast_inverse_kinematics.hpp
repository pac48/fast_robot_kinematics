#pragma once

#include "memory"

#include <Eigen/Core>
#include "LBFGS.h"
#include <Eigen/Dense>

#include "fast_kinematics_joint_data_length.hpp"
#include "kinematics_interface.hpp"

namespace fast_fk::internal {

#ifdef FAST_FK_USE_IK
    class InverseKinematics {
    public:
        Eigen::Matrix<float, 3, 3> target_rot_;
        Eigen::Vector<float, 3> target_pose_;
        std::array<std::array<float, internal::joint_data_length>, FAST_FK_NUMBER_OF_JOINTS> joint_data = {0};
        std::unique_ptr<LBFGSpp::LBFGSSolver<float>> solver;
        LBFGSpp::LBFGSParam<float> param;

        InverseKinematics(const Eigen::Matrix<float, 3, 3> &target_rot,
                          const Eigen::Vector<float, 3> &target_pose);

        float operator()(const Eigen::VectorX<float> &q, Eigen::VectorX<float> &grad);

        fk_interface::IKSolverStats inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess);

    };
#else
    class InverseKinematics {
    public:
        InverseKinematics(const Eigen::Matrix<float, 3, 3> &target_rot,
                          const Eigen::Vector<float, 3> &target_pose);

        float operator()(const Eigen::VectorX<float> &q, Eigen::VectorX<float> &grad);

        fk_interface::IKSolverStats inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess);
};
#endif
}
