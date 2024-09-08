#pragma once

#include "memory"

#include <Eigen/Core>
#include "LBFGS.h"
#include <Eigen/Dense>

#include "kinematics_interface.hpp"

namespace fast_fk {
    namespace internal {
        // input_data: sin(t) cos(t)  px py pz R11, R12, R13...
        void forward_kinematics_internal(float *input_data, size_t size);

        constexpr size_t joint_data_length = 16;

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
        fast_fk::internal::InverseKinematics fun;
        std::array<std::array<float, internal::joint_data_length>, FAST_FK_NUMBER_OF_JOINTS> joint_data = {0};
    };

}
