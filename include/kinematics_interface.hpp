#pragma once

#include <concepts>
#include <iostream>

#include <Eigen/Core>

namespace fk_interface {

    struct IKSolverStats {
        float fx = 0;
        int niter = 0;
        float grad_norm = 0;
        bool success = false;
        std::string what;
    };

    template<typename T>
    concept KinematicInterfaceConstraint = requires(T obj, size_t ind, float value,
                                                    const Eigen::Vector<float, T::get_num_joints()> &values,
                                                    Eigen::Vector<float, T::get_num_joints()> &values_non_const,
                                                    Eigen::VectorX<float> &q_guess) {
        { T::get_num_joints() };
        { obj.set_joint(ind, value) };
        { obj.set_joints(values) };
        { obj.set_joint(ind, value, value) };
        { obj.set_joints(values, values) };
        { obj.get_joint(ind) };
        { obj.get_joints(values_non_const) };
    };


    template<typename T>
    concept ForwardKinematicInterfaceConstraint = requires(T obj, size_t ind, Eigen::Matrix<float, 4, 4> &transform) {
        { obj.get_frame(ind, transform) };
        { obj.forward_kinematics() };
    };

    template<typename T>
    concept InverseInterfaceConstraint = requires(T obj, Eigen::VectorX<float> &q_guess,
                                                  Eigen::Matrix<float, 4, 4> &transform) {
        { obj.inverse_kinematics(transform, q_guess) } -> std::same_as<IKSolverStats>;
    };

    template<typename KI> requires KinematicInterfaceConstraint<KI> && ForwardKinematicInterfaceConstraint<KI>
    struct ForwardKinematicsInterface : KI {
    };


    template<typename IK> requires KinematicInterfaceConstraint<IK> && ForwardKinematicInterfaceConstraint<IK> &&
                                   InverseInterfaceConstraint<IK>
    struct InverseKinematicsInterface : IK {
    };

}
