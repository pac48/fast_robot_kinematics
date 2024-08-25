#pragma once

#include <concepts>
#include <iostream>

#include <Eigen/Core>

namespace fk_interface {

    template<typename T>
    concept KinematicInterface = requires(T obj, size_t ind, float value,
                                          const Eigen::Vector<float, T::get_num_joints()> &values,
                                          Eigen::Vector<float, T::get_num_joints()> &values_non_const,
                                          Eigen::VectorX<float> &q_guess,
                                          Eigen::Matrix<float, 4, 4> &transform) {
        { T::get_num_joints() };
        { obj.set_joint(ind, value) };
        { obj.set_joints(values) };
        { obj.set_joint(ind, value, value) };
        { obj.set_joints(values, values) };
        { obj.get_joint(ind) };
        { obj.get_joints(values_non_const) };
        { obj.get_frame(ind, transform) };
        { obj.forward_kinematics() };
        { obj.inverse_kinematics(transform, q_guess) };
    };


    struct IKSolverStats {
        float fx = 0;
        int niter = 0;
        float grad_norm = 0;
        bool success = false;
        std::string what;
    };

    template<KinematicInterface KI>
    struct JointDataInterface {
        KI kinematic_interface;

        static constexpr size_t get_num_joints() {
            return KI::get_num_joints();
        }

        void set_joint(size_t ind, float value) {
            kinematic_interface.set_joint(ind, value);
        }

        void set_joints(const Eigen::Vector<float, KI::get_num_joints()> &values) {
            kinematic_interface.set_joints(values);
        }

        void set_joint(size_t ind, float sin_t, float cos_t) {
            kinematic_interface.set_joint(ind, sin_t, cos_t);
        }

        void set_joints(const Eigen::Vector<float, KI::get_num_joints()> &sin_values,
                        const Eigen::Vector<float, KI::get_num_joints()> &cos_values) {
            kinematic_interface.set_joints(sin_values, cos_values);
        }

        [[nodiscard]] float get_joint(size_t ind) const {
            return kinematic_interface.get_joint(ind);
        }

        void get_joints(Eigen::Vector<float, KI::get_num_joints()> &values) const {
            kinematic_interface.get_joints(values);
        }

        void get_frame(size_t index, Eigen::Matrix<float, 4, 4> &transform) const {
            kinematic_interface.get_frame(index, transform);
        }

        void forward_kinematics() {
            kinematic_interface.forward_kinematics();
        }

        IKSolverStats inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess) {
            return kinematic_interface.inverse_kinematics(transform, q_guess);
        }

        static_assert(KinematicInterface<JointDataInterface<KI>>);
    };


}
