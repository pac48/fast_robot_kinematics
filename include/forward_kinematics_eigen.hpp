#ifndef FAST_FORWARD_KINEMATICS_HPP
#define FAST_FORWARD_KINEMATICS_HPP

#include <Eigen/Core>

namespace fast_fk {
  namespace internal {
    void forward_kinematics_internal(double *input_data);
    constexpr size_t joint_data_length = 17;
  }

  struct JointData {
    std::array<double, internal::joint_data_length> joint_data;

    void set_joint(double value) {
      joint_data[0] = std::sin(value);
      joint_data[1] = std::cos(value);
    }

    void set_joints(const double *values) {
      for (auto i = 0; i < FAST_FK_NUMBER_OF_JOINTS; ++i) {
        joint_data[i * internal::joint_data_length] = std::sin(values[i]);
        joint_data[i * internal::joint_data_length + 1] = std::cos(values[i]);
      }
    }

    double get_joint(size_t ind) {
      return asin(joint_data[ind * 17]);
    }

    void get_joints(double *values) {
      for (auto i = 0; i < FAST_FK_NUMBER_OF_JOINTS; ++i) {
        values[i] = asin(joint_data[i * internal::joint_data_length]);
      }
    }

    void set_joint(double sin_t, double cos_t) {
      joint_data[0] = sin_t;
      joint_data[1] = cos_t;
    }

    void set_joints(const double *sin_values, const double *cos_values) {
      for (auto i = 0; i < FAST_FK_NUMBER_OF_JOINTS; ++i) {
        joint_data[i * internal::joint_data_length] = sin_values[i];
        joint_data[i * internal::joint_data_length + 1] = cos_values[i];
      }
    }

    void get_frame(){

    }
  };


  template<std::size_t SIZE>
  void forward_kinematics(std::array<JointData, SIZE> &input_data) {
    static_assert(SIZE == FAST_FK_NUMBER_OF_JOINTS,
                  "`forward_kinematics` called with the wrong number of joints. Hint: FAST_FK_NUMBER_OF_JOINTS defines the correct number of joints.");
    internal::forward_kinematics_internal(input_data.data()->joint_data.data());
  }

}

#endif //FAST_FORWARD_KINEMATICS_HPP
