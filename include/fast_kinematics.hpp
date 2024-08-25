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
                              const Eigen::Vector<float, 3> &target_pose) : target_rot_{target_rot},
                                                                            target_pose_{target_pose} {}

            float operator()(const Eigen::VectorX<float> &q, Eigen::VectorX<float> &grad);

        };
    }


    struct JointData {
        static constexpr size_t get_num_joints() {
            return FAST_FK_NUMBER_OF_JOINTS;
        }

        Eigen::Matrix<float, 3, 3> target_rot;
        Eigen::Vector<float, 3> target_pose;
        std::unique_ptr<LBFGSpp::LBFGSSolver<float>> solver;
        LBFGSpp::LBFGSParam<float> param;
        fast_fk::internal::InverseKinematics fun;

        JointData() : target_pose{Eigen::Vector<float, 3>::Zero()}, target_rot{Eigen::Matrix<float, 3, 3>::Zero()},
                      fun(target_rot, target_pose) {
            param.epsilon = 1E-3;
            param.epsilon_rel = 1E-3;
            param.max_iterations = 30;

            solver = std::make_unique<LBFGSpp::LBFGSSolver<float>>(param);
        }


        void set_joint(size_t ind, float value) {
            joint_data[ind][0] = std::sin(value);
            joint_data[ind][1] = std::cos(value);
        }

        void set_joints(const Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &values) {
#pragma unroll
            for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
                joint_data[ind][0] = std::sin(values[ind]);
                joint_data[ind][1] = std::cos(values[ind]);
            }
        }

        void set_joint(size_t ind, float sin_t, float cos_t) {
            joint_data[ind][0] = sin_t;
            joint_data[ind][1] = cos_t;
        }

        void set_joints(const float *sin_values, const float *cos_values) {
#pragma unroll
            for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
                joint_data[ind][0] = sin_values[ind];
                joint_data[ind][1] = cos_values[ind];
            }
        }

        void set_joints(const Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &sin_values,
                        const Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &cos_values) {
#pragma unroll
            for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
                joint_data[ind][0] = sin_values[ind];
                joint_data[ind][1] = cos_values[ind];
            }
        }


        [[nodiscard]] float get_joint(size_t ind) const {
            return atan2f(joint_data[ind][0], joint_data[ind][1]);
        }

        void get_joints(Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &values) const {
#pragma unroll
            for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
                values[ind] = atan2f(joint_data[ind][0], joint_data[ind][1]);
            }
        }


        void get_frame(size_t index, Eigen::Matrix<float, 4, 4> &transform) const {
            transform(0, 3) = joint_data[index][2];
            transform(1, 3) = joint_data[index][3];
            transform(2, 3) = joint_data[index][4];

            transform(0, 0) = joint_data[index][5];
            transform(0, 1) = joint_data[index][6];
            transform(0, 2) = joint_data[index][7];

            transform(1, 0) = joint_data[index][8];
            transform(1, 1) = joint_data[index][9];
            transform(1, 2) = joint_data[index][10];

            transform(2, 0) = joint_data[index][11];
            transform(2, 1) = joint_data[index][12];
            transform(2, 2) = joint_data[index][13];

            transform(3, 0) = 0.0;
            transform(3, 1) = 0.0;
            transform(3, 2) = 0.0;
            transform(3, 3) = 1.0;
        }

        void forward_kinematics() {
            internal::forward_kinematics_internal(joint_data.data()->data(),
                                                  joint_data.size() * internal::joint_data_length);
        }

        fk_interface::IKSolverStats
        inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess) {
            target_rot = transform.block<3, 3>(0, 0);
            target_pose(0) = transform(0, 3);
            target_pose(1) = transform(1, 3);
            target_pose(2) = transform(2, 3);
            float fx = 1E10;
            int niter;

            try {
                niter = solver->minimize(fun, q_guess, fx);
            } catch (const std::runtime_error &e) {
                return {fx, niter, solver->final_grad_norm(), false, e.what()};
            }

            return {fx, niter, solver->final_grad_norm(), true, ""};
        }

        std::array<std::array<float, internal::joint_data_length>, FAST_FK_NUMBER_OF_JOINTS> joint_data = {0};
    };


}
