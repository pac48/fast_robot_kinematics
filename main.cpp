#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/string.hpp"

#include "model.h"
#include "parse_urdf.h"
#include "kinematics.h"




void fillTFMessage(const Model& model, rclcpp::Clock& clock, tf2_msgs::msg::TFMessage& msg){
  for (auto& joint : model.joints) {
    geometry_msgs::msg::TransformStamped tmp;

    tmp.header.stamp = clock.now();
    tmp.child_frame_id = joint.child_link;
    tmp.header.frame_id = "base_link";//joint.parent_link;

    tmp.transform.translation.x = joint.transform(0, 3);
    tmp.transform.translation.y = joint.transform(1, 3);
    tmp.transform.translation.z = joint.transform(2, 3);

    Eigen::Matrix3d rot = joint.transform.block<3, 3>(0, 0);
    Eigen::Quaternion<double> quat(rot);
    tmp.transform.rotation.w = quat.w();
    tmp.transform.rotation.x = quat.x();
    tmp.transform.rotation.y = quat.y();
    tmp.transform.rotation.z = quat.z();

    msg.transforms.push_back(tmp);

  }

}

int main(int arc, char **argv) {

  std::string urdf_file = "/home/paul/CLionProjects/forward_kinematics/robot.urdf";
  Model model;
  parseURDF(urdf_file, model);
  model.buildTree("base_link", "wrist_3_joint");



  // ROS
  rclcpp::init(arc, argv);
  auto node = std::make_shared<rclcpp::Node>("kinematics");
  auto tf_pub = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(node, "/tf", 1);
  auto desc_pub = rclcpp::create_publisher<std_msgs::msg::String>(node, "/robot_description", 1);
  rclcpp::Clock clock(RCL_ROS_TIME);
  std_msgs::msg::String msg2;
  msg2.data = getRobotDescription(urdf_file);

  desc_pub->publish(msg2);
  while (rclcpp::ok()) {
    // add joint values
    for (auto joint : model.chain){
      joint->val[0] += 0.01;
    }

    forwardPosition(model);
    tf2_msgs::msg::TFMessage tf_msg;
    fillTFMessage(model, clock, tf_msg);
    tf_pub->publish(tf_msg);

    usleep(10000);
  }

  return 0;
}
