#pragma once

#include "array"
#include "cmath"

namespace fast_fk {

  struct JointData {
    std::array<double, 17> joint_data;

    void set_joint(double value) {
      joint_data[0] = std::sin(value);
      joint_data[1] = std::cos(value);
    }

    void set_joint(double sin_t, double cos_t) {
      joint_data[0] = sin_t;
      joint_data[1] = cos_t;
    }
  };

  namespace internal {
    template<std::size_t SIZE>
    void forward_kinematics_internal(std::array<JointData, SIZE> &input_data);
  }

  template<std::size_t SIZE>
  void forward_kinematics(std::array<JointData, SIZE> &input_data) {
    static_assert(SIZE == FAST_FK_NUMBER_OF_JOINTS,
                  "`forward_kinematics` called with the wrong number of joints. Hint: FAST_FK_NUMBER_OF_JOINTS defines the correct number of joints.");
    internal::forward_kinematics_internal(input_data);
  }

}
