//
// Created by paul on 6/30/22.
//

#ifndef FORWARD_KINEMATIC_PARSE_URDF_H
#define FORWARD_KINEMATIC_PARSE_URDF_H

#include "tinyxml2.h"
#include "vector"
#include "queue"
#include "sstream"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/string.hpp"
#include "model.h"



void parseOrigin(tinyxml2::XMLElement *element, Eigen::Matrix<double,4,4>& origin){
  origin.setIdentity();

  auto tmp = element->FindAttribute("rpy");
  if (tmp){
    auto rpy_str = std::string(tmp->Value());
    std::stringstream ss(rpy_str);
    double roll, pitch, yaw;
    ss >> roll >> pitch >> yaw;
//    roll = 3.14/2;
//    pitch = 3.14/2;
//    yaw = 3.14/2;
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix<double,3,3> r = rollAngle.matrix();
    Eigen::Matrix<double,3,3> p = pitchAngle.matrix();
    Eigen::Matrix<double,3,3> y = yawAngle.matrix();
    origin.block<3,3>(0,0) = ( yawAngle*pitchAngle *rollAngle).matrix();
  }
  tmp = element->FindAttribute("xyz");
  if (tmp) {
    auto xyz_str = std::string(element->FindAttribute("xyz")->Value());
    std::stringstream ss = std::stringstream(xyz_str);
    ss >> origin(0,3) >> origin(1,3) >> origin(2,3);
  }
}

void parseInertia(tinyxml2::XMLElement *element, Inertia& inertia) {
  inertia.ixx = std::stod(std::string(element->FindAttribute("ixx")->Value()));
  inertia.ixy = std::stod(std::string(element->FindAttribute("ixy")->Value()));
  inertia.ixz = std::stod(std::string(element->FindAttribute("ixz")->Value()));
  inertia.iyy = std::stod(std::string(element->FindAttribute("iyy")->Value()));
  inertia.iyz = std::stod(std::string(element->FindAttribute("iyz")->Value()));
  inertia.izz = std::stod(std::string(element->FindAttribute("izz")->Value()));
}

void parseInertial(tinyxml2::XMLElement *element, Inertial& inertial) {
  for (auto child_element = element->FirstChildElement(); child_element; child_element = child_element->NextSiblingElement()) {
    if (strcmp(child_element->Name(), "mass") == 0) {
      inertial.mass = std::stod(child_element->FindAttribute("value")->Value());
    } else if (strcmp(child_element->Name(), "origin") == 0) {
      parseOrigin(child_element, inertial.origin);
    }else if (strcmp(child_element->Name(), "inertia") == 0) {
      parseInertia(child_element, inertial.inertia);
    }
  }
}

void parseLink(tinyxml2::XMLElement *element, Link& link) {
  link.name = element->FindAttribute("name")->Value();

  for (auto child_element = element->FirstChildElement(); child_element; child_element = child_element->NextSiblingElement()) {
    if (strcmp(child_element->Name(), "inertial") == 0) {
      parseInertial(child_element, link.inertia);
    }
  }
}

void parseJoint(tinyxml2::XMLElement *element, Joint& joint) {
  joint.name = element->FindAttribute("name")->Value();
  auto tmp_str = element->FindAttribute("type")->Value();
  if (strcmp(tmp_str, "fixed") == 0) {
    joint.joint_type = JointType::FIXED;
  } else if (strcmp(tmp_str, "revolute") == 0) {
    joint.joint_type = JointType::REVOLUTE;
  } else if (strcmp(tmp_str, "prismatic") == 0) {
    joint.joint_type = JointType::PRISMATIC;
  }else if (strcmp(tmp_str, "continuous") == 0) {
    joint.joint_type = JointType::CONTINUOUS;
  } else{
    printf("type (%s) not supported", tmp_str);
    assert(0);
  }

  for (auto child_element = element->FirstChildElement(); child_element; child_element = child_element->NextSiblingElement()) {
    if (strcmp(child_element->Name(), "origin") == 0) {
      parseOrigin(child_element, joint.transform_local);
    } else if (strcmp(child_element->Name(), "parent") == 0) {
      joint.parent_link = child_element->FindAttribute("link")->Value();
    } else if (strcmp(child_element->Name(), "child") == 0) {
      joint.child_link = child_element->FindAttribute("link")->Value();
    }
  }
}

void parseURDF(tinyxml2::XMLElement *element, Model& model) {
  for (auto child_element = element->FirstChildElement(); child_element; child_element = child_element->NextSiblingElement()) {
    if (strcmp(child_element->Name(), "joint") == 0) {
      Joint joint;
      parseJoint(child_element, joint);
      model.joints.push_back(joint);
    } else if (strcmp(child_element->Name(), "link") == 0) {
      Link link;
      parseLink(child_element, link);
      model.links.push_back(link);
    }
  }
}

void parseURDF(std::string file_name, Model& model){
  tinyxml2::XMLDocument doc;
  doc.LoadFile(file_name.c_str());
  tinyxml2::XMLElement *root = (tinyxml2::XMLElement *) doc.RootElement();
  parseURDF(root, model);
}

std::string getRobotDescription(std::string file_name){
  tinyxml2::XMLDocument doc;
  doc.LoadFile(file_name.c_str());
  tinyxml2::XMLPrinter printer;
  doc.Accept(&printer);
  return std::string(printer.CStr());
}






#endif //FORWARD_KINEMATIC_PARSE_URDF_H
