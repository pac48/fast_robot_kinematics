#ifndef FAST_FORWARD_KINEMATICS_HPP
#define FAST_FORWARD_KINEMATICS_HPP

#include "array"
#include "cmath"

namespace fast_fk {
  namespace internal {
    // input_data: sin(t) cos(t)  px py pz R11, R12, R13...
    void forward_kinematics_internal(double *input_data, size_t size);

    constexpr size_t joint_data_length = 17;
  }

  struct JointData {
    std::array<std::array<double, internal::joint_data_length>, FAST_FK_NUMBER_OF_JOINTS> joint_data = {0};

    void set_joint(size_t ind, double value) {
      joint_data[ind][0] = std::sin(value);
      joint_data[ind][1] = std::cos(value);
    }

    void set_joints(const double *values) {
      for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
        joint_data[ind][0] = std::sin(values[ind]);
        joint_data[ind][1] = std::cos(values[ind]);
      }
    }

    void set_joint(size_t ind, double sin_t, double cos_t) {
      joint_data[ind][0] = sin_t;
      joint_data[ind][1] = cos_t;
    }

    void set_joints(const double *sin_values, const double *cos_values) {
      for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
        joint_data[ind][0] = sin_values[ind];
        joint_data[ind][1] = cos_values[ind];
      }
    }

    [[nodiscard]] double get_joint(size_t ind) const {
      return asin(joint_data[ind][0]);
    }

    void get_joints(double *values) const {
      for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
        values[ind] = asin(joint_data[ind][0]);
      }
    }
  };

  void forward_kinematics(JointData &input_data) {
    internal::forward_kinematics_internal(input_data.joint_data.data()->data(), input_data.joint_data.size()*internal::joint_data_length);
  }

}

#endif //FAST_FORWARD_KINEMATICS_HPP
