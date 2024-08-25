#pragma once

#include "LBFGS.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "forward_kinematics_eigen.hpp"

namespace fast_fk {

    class InverseKinematics {
    public:
        Eigen::Matrix<float, 3, 3> target_rot_;
        Eigen::Vector<float, 3> target_pose_;
        JointData joint_data;


        InverseKinematics(Eigen::Matrix<float, 3, 3> target_rot,
                          const Eigen::Vector<float, 3> &target_pose);

        float operator()(const Eigen::VectorX<float> &q, Eigen::VectorX<float> &grad);

    };
}
