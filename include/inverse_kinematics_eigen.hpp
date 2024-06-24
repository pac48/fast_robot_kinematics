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

        Eigen::Vector3d target_x_axis_;
        Eigen::Vector3d target_y_axis_;
        Eigen::Vector3d target_z_axis_;

        Eigen::Vector3d target_x_;
        Eigen::Vector3d target_y_;
        Eigen::Vector3d target_z_;

        double input_data[internal::joint_data_length * FAST_FK_NUMBER_OF_JOINTS];


        InverseKinematics(Eigen::Matrix<double, 3, 3> target_rot,
                          const Eigen::Vector<double, 3> &target_pose);

        double operator()(const Eigen::VectorXd &q, Eigen::VectorXd &grad);

    };
}
