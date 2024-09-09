#pragma once

#include "memory"

#include <Eigen/Core>

#include "fast_kinematics.hpp"
#include "fast_inverse_kinematics.hpp"

namespace fast_fk {
    namespace internal {
        // input_data: sin(t) cos(t)  px py pz R11, R12, R13...
        void forward_kinematics_internal(float *input_data, size_t size);
    }

    JointData::JointData() : target_pose{Eigen::Vector<float, 3>::Zero()},
                             target_rot{Eigen::Matrix<float, 3, 3>::Zero()},
                             fun{std::make_unique<internal::InverseKinematics>(target_rot, target_pose)} {

    }


    void JointData::set_joint(size_t ind, float value) {
        throw std::logic_error("not implemented");
//        joint_data[ind][0] = std::sin(value);
//        joint_data[ind][1] = std::cos(value);
    }

    void JointData::set_joints(size_t  batch_ind, const Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &values) {
#pragma unroll
        for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
            joint_data[batch_ind][ind][0] = values[ind];
            joint_data[batch_ind][ind][1] = values[ind];
        }
    }

    void JointData::set_joints(const Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &values) {

        throw std::logic_error("not implemented");
    }



    void JointData::set_joint(size_t ind, float sin_t, float cos_t) {
        throw std::logic_error("not implemented");
//        joint_data[ind][0] = sin_t;
//        joint_data[ind][1] = cos_t;
    }

    void JointData::set_joints(const float *sin_values, const float *cos_values) {
        throw std::logic_error("not implemented");
        //#pragma unroll
//        for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
//            joint_data[ind][0] = sin_values[ind];
//            joint_data[ind][1] = cos_values[ind];
//        }
    }

    void JointData::set_joints(const Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &sin_values,
                               const Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &cos_values) {
        throw std::logic_error("not implemented");
        //#pragma unroll
//        for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
//            joint_data[ind][0] = sin_values[ind];
//            joint_data[ind][1] = cos_values[ind];
//        }
    }


    [[nodiscard]] float JointData::get_joint(size_t ind) const {
        throw std::logic_error("not implemented");
        //        return atan2f(joint_data[ind][0], joint_data[ind][1]);
    }

    void JointData::get_joints(Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS> &values) const {
        throw std::logic_error("not implemented");
//#pragma unroll
//        for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
//            values[ind] = atan2f(joint_data[ind][0], joint_data[ind][1]);
//        }
    }


    void JointData::get_frame(size_t index, Eigen::Matrix<float, 4, 4> &transform) const {
        throw std::logic_error("not implemented");
//        transform(0, 3) = joint_data[index][2];
//        transform(1, 3) = joint_data[index][3];
//        transform(2, 3) = joint_data[index][4];
//
//        transform(0, 0) = joint_data[index][5];
//        transform(0, 1) = joint_data[index][6];
//        transform(0, 2) = joint_data[index][7];
//
//        transform(1, 0) = joint_data[index][8];
//        transform(1, 1) = joint_data[index][9];
//        transform(1, 2) = joint_data[index][10];
//
//        transform(2, 0) = joint_data[index][11];
//        transform(2, 1) = joint_data[index][12];
//        transform(2, 2) = joint_data[index][13];
//
//        transform(3, 0) = 0.0;
//        transform(3, 1) = 0.0;
//        transform(3, 2) = 0.0;
//        transform(3, 3) = 1.0;
    }

    void JointData::forward_kinematics() {
        internal::forward_kinematics_internal(joint_data.data()->data()->data(),
                                              joint_data.size() * joint_data.data()->size() *internal::joint_data_length);
    }

    fk_interface::IKSolverStats
    JointData::inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess) {
        throw std::logic_error("not implemented");
//        return fun->inverse_kinematics(transform, q_guess);
    }
}