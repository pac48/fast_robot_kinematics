#include "kdl_kinematics.hpp"

namespace kdl_impl {

    void JointData::get_frame(size_t index, Eigen::Matrix<float, 4, 4> &tf) const {
        tf = transform;
    }

    // taken from https://github.com/ros/geometry/blob/noetic-devel/eigen_conversions/src/eigen_kdl.cpp
    void transformKDLToEigen(const KDL::Frame &k, Eigen::Matrix<float, 4, 4> &e) {
        // translation
        for (unsigned int i = 0; i < 3; ++i)
            e(i, 3) = k.p[i];

        // rotation matrix
        for (unsigned int i = 0; i < 9; ++i)
            e(i / 3, i % 3) = k.M.data[i];

        // "identity" row
        e(3, 0) = 0.0;
        e(3, 1) = 0.0;
        e(3, 2) = 0.0;
        e(3, 3) = 1.0;
    }

    void transformEigenToKDL(Eigen::Matrix<float, 4, 4> &e, KDL::Frame &k) {
        for (unsigned int i = 0; i < 3; ++i)
            k.p[i] = e(i, 3);
        for (unsigned int i = 0; i < 9; ++i)
            k.M.data[i] = e(i / 3, i % 3);
    }

    void JointData::forward_kinematics() {
        // create forward kinematics solver
        fk_pos_solver->JntToCart(q, frame, ee_ind);
        transformKDLToEigen(frame, transform);
        int o = 0;
    }

    fk_interface::IKSolverStats
    JointData::inverse_kinematics(Eigen::Matrix<float, 4, 4> &tf, Eigen::VectorX<float> &q_guess) {
        for (int ind = 0; ind < get_num_joints(); ++ind) {
            q.data[ind] = q_guess[ind];
        }
        transformEigenToKDL(tf, frame);

        auto ret = ik_solver_lma->CartToJnt(q, frame, q_out);

        return {-1.0, -1, -1, ret == 0, ""};
    }
}