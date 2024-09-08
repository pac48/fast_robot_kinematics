#pragma once

#include "memory"

#include <Eigen/Core>
#include "LBFGS.h"
#include <Eigen/Dense>

#include "kinematics_interface.hpp"
#include "fast_inverse_kinematics.hpp"
#include "fast_kinematics_joint_data_length.hpp"

namespace fast_fk {
    struct JointData {

        static constexpr size_t get_num_joints() {
            return FAST_FK_NUMBER_OF_JOINTS;
        }

        JointData();

        void set_joint(size_t ind, float value);

        void set_joints(const Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &values);

        void set_joint(size_t ind, float sin_t, float cos_t);

        void set_joints(const float *sin_values, const float *cos_values);

        void set_joints(const Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &sin_values,
                        const Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &cos_values);


        [[nodiscard]] float get_joint(size_t ind) const;

        void get_joints(Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &values) const;


        void get_frame(size_t index, Eigen::Matrix<float, 4, 4> &transform) const;

        void forward_kinematics();

        fk_interface::IKSolverStats
        inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess);

        Eigen::Matrix<float, 3, 3> target_rot;
        Eigen::Vector<float, 3> target_pose;
        std::unique_ptr<LBFGSpp::LBFGSSolver<float>> solver;
        LBFGSpp::LBFGSParam<float> param;
        std::unique_ptr<fast_fk::internal::InverseKinematics> fun;
        std::array<std::array<float, internal::joint_data_length>, FAST_FK_NUMBER_OF_JOINTS> joint_data = {0};
    };

}
