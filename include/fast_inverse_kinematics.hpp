#pragma once

#include "memory"

#include <Eigen/Core>
#include <Eigen/Dense>

#ifdef FAST_FK_USE_IK
#include "LBFGS.h"

#include "kinematics_interface.hpp"

namespace fast_fk::internal {
    class InverseKinematics {
    public:
        Eigen::Matrix<float, 3, 3> target_rot_;
        Eigen::Vector<float, 3> target_pose_;
        std::array<std::array<float, 16>, FAST_FK_NUMBER_OF_JOINTS> joint_data = {0};
        std::unique_ptr<LBFGSpp::LBFGSSolver<float>> solver;
        LBFGSpp::LBFGSParam<float> param;

        InverseKinematics(const Eigen::Matrix<float, 3, 3> &target_rot,
                          const Eigen::Vector<float, 3> &target_pose);

        float operator()(const Eigen::VectorX<float> &q, Eigen::VectorX<float> &grad);

        fk_interface::IKSolverStats inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess);

    };
    }
#else
namespace fast_fk::internal {
    class InverseKinematics {
    public:
        InverseKinematics(const Eigen::Matrix<float, 3, 3> &target_rot,
                          const Eigen::Vector<float, 3> &target_pose);

        float operator()(const Eigen::VectorX<float> &q, Eigen::VectorX<float> &grad);

        fk_interface::IKSolverStats
        inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess);
    };
}
#endif
