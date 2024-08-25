#pragma once

#include "LBFGS.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "forward_kinematics_eigen.hpp"

namespace fast_fk {

    class InverseKinematics {
    public:
        Eigen::Matrix<double, 3, 3> target_rot_;
        Eigen::Vector<double, 3> target_pose_;
        JointData joint_data;


        InverseKinematics(Eigen::Matrix<double, 3, 3> target_rot,
                          const Eigen::Vector<double, 3> &target_pose);

        double operator()(const Eigen::VectorXd &q, Eigen::VectorXd &grad);

    };
}
