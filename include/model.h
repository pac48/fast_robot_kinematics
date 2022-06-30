//
// Created by paul on 6/30/22.
//

#ifndef FORWARD_KINEMATIC_MODEL_H
#define FORWARD_KINEMATIC_MODEL_H

#include <Eigen/Core>

enum JointType {
  FIXED,
  REVOLUTE,
  PRISMATIC,
  CONTINUOUS
};

struct Inertia {
  double ixx = 1.0;
  double ixy = 0.0;
  double ixz = 0.0;
  double iyy = 1.0;
  double iyz = 0.0;
  double izz = 1.0;
};

struct Inertial {
  double mass;
  Eigen::Matrix<double, 4, 4> origin;
  Inertia inertia;

  Inertial() {
    origin.setIdentity();
    mass = 1.0;
  }
};

struct Link {
  std::string name;
  Inertial inertia;
};


struct Joint {
  std::string name;
  JointType joint_type;
  double val[3] = {0};
  Eigen::Matrix<double, 4, 4> transform_local;
  Eigen::Matrix<double, 4, 4> transform;
  std::string parent_link;
  std::string child_link;
  Joint *parent = nullptr;
};


struct Model {
  std::vector<Link> links;
  std::vector<Joint> joints;
  Joint *root;
  std::vector<Joint *> chain;

  void buildTree(const std::string &base, const std::string &end_effector) {
    std::unordered_map<std::string, Link *> name_link_map;
    for (auto link: links) {
      name_link_map[link.name] = &link;
    }

    std::unordered_map<std::string, Joint *> name_joint_map;
    for (auto &joint: joints) {
      name_joint_map[joint.name] = &joint;
    }

    std::unordered_map<std::string, std::vector<Joint *>> joint_parent_map;
    for (auto &joint: joints) {
      joint_parent_map[joint.child_link].push_back(&joint);
    }

    root = name_joint_map[base];

    for (auto &joint: joints) {
      auto children = joint_parent_map[joint.parent_link];
      for (auto child: children) {
        joint.parent = child;
      }
    }

    Joint* tmp_joint = name_joint_map[end_effector];
    while (tmp_joint) {
      chain.push_back(tmp_joint);
      tmp_joint = tmp_joint->parent;
    }

  }
};


#endif //FORWARD_KINEMATIC_MODEL_H
