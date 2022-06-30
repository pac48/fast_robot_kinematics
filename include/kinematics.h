//
// Created by paul on 6/30/22.
//

#ifndef FORWARD_KINEMATIC_KINEMATICS_H
#define FORWARD_KINEMATIC_KINEMATICS_H

#include <Eigen/Core>
#include "model.h"

void forwardPosition(Model& model){
  Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();
  for (int i = model.chain.size() - 1; i >= 0; i--) {
    Joint *joint = model.chain[i];
    Eigen::Matrix<double, 4, 4> T_joint = Eigen::Matrix<double, 4, 4>::Identity();
    if (joint->joint_type == JointType::REVOLUTE){
      Eigen::AngleAxisd rot(joint->val[0], Eigen::Vector3d::UnitZ());
      T_joint.block<3,3>(0,0) = rot.matrix();
    }
    T = T * joint->transform_local*T_joint;
    joint->transform = T;
  }
}

#endif //FORWARD_KINEMATIC_KINEMATICS_H
