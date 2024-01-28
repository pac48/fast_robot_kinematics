#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "model.h"
#include "parse_urdf.h"
#include "kinematics.h"


int main(int arc, char **argv) {

  std::string urdf_file = "/home/paul/CLionProjects/forward_kinematics/robot.urdf";
  Model model;
  parseURDF(urdf_file, model);
  model.buildTree("base_link", "wrist_3_joint");


  for (auto joint_ind: model.chain) {
    model.joints[joint_ind].val[0] += 0.01;
  }

  forwardPosition(model);

  return 0;
}
