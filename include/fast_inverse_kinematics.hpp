#pragma once

#include "memory"

#include <Eigen/Core>
#include "LBFGS.h"
#include <Eigen/Dense>

#include "fast_kinematics.hpp"
#include "fast_kinematics_joint_data_length.hpp"

namespace fast_fk::internal {
    class InverseKinematics {
    public:
        const Eigen::Matrix<float, 3, 3> &target_rot_;
        const Eigen::Vector<float, 3> &target_pose_;
        std::array<std::array<float, internal::joint_data_length>, FAST_FK_NUMBER_OF_JOINTS> joint_data = {0};

        InverseKinematics(const Eigen::Matrix<float, 3, 3> &target_rot,
                          const Eigen::Vector<float, 3> &target_pose);

        float operator()(const Eigen::VectorX<float> &q, Eigen::VectorX<float> &grad);

    };
}
