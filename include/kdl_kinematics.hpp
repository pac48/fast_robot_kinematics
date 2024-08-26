#pragma once

#include "memory"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "chainfksolverpos_recursive.hpp"
#include "chainfksolvervel_recursive.hpp"
#include "chainjnttojacsolver.hpp"
#include "treejnttojacsolver.hpp"
#include "kdl_parser/kdl_parser.hpp"

#include "kinematics_interface.hpp"


#define STRINGIZE(x) #x
#define STRINGIZE_VALUE_OF(x) STRINGIZE(x)

namespace kdl_impl {


    struct JointData {
        static constexpr size_t get_num_joints() {
            return NUMBER_OF_JOINTS;
        }

        JointData() {
           std::string urdf_file = STRINGIZE_VALUE_OF(URDF_FILE);

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

        }

        fk_interface::IKSolverStats
        inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess) {

        }

        std::array<std::array<float, internal::joint_data_length>, FAST_FK_NUMBER_OF_JOINTS> joint_data = {0};
    };


}
