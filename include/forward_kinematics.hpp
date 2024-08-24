#pragma once

#include "array"
#include "cmath"

namespace fast_fk {
  namespace internal {
    // input_data: sin(t) cos(t)  px py pz R11, R12, R13...
    void forward_kinematics_internal(float *input_data, size_t size);

    constexpr size_t joint_data_length = 16;
  }

  struct JointData {
    std::array<std::array<float, internal::joint_data_length>, FAST_FK_NUMBER_OF_JOINTS> joint_data = {0};

    void set_joint(size_t ind, float value) {
      joint_data[ind][0] = std::sin(value);
      joint_data[ind][1] = std::cos(value);
    }

    void set_joints(const float *values) {
      for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
        joint_data[ind][0] = std::sin(values[ind]);
        joint_data[ind][1] = std::cos(values[ind]);
      }
    }

    void set_joint(size_t ind, float sin_t, float cos_t) {
      joint_data[ind][0] = sin_t;
      joint_data[ind][1] = cos_t;
    }

    void set_joints(const float *sin_values, const float *cos_values) {
      for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
        joint_data[ind][0] = sin_values[ind];
        joint_data[ind][1] = cos_values[ind];
      }
    }

    [[nodiscard]] float get_joint(size_t ind) const {
      return asin(joint_data[ind][0]);
    }

    void get_joints(float *values) const {
      for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
        values[ind] = asin(joint_data[ind][0]);
      }
    }
  };

  void forward_kinematics(JointData &input_data) {
    internal::forward_kinematics_internal(input_data.joint_data.data()->data(), input_data.joint_data.size()*internal::joint_data_length);
  }

}