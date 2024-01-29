#ifndef FAST_FORWARD_KINEMATICS_HPP
#define FAST_FORWARD_KINEMATICS_HPP

#include <Eigen/Core>

namespace fast_fk {
  namespace internal {
    // input_data: sin(t) cos(t)  px py pz R11, R12, R13...
    void forward_kinematics_internal(double *input_data);

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

    void set_joints(const Eigen::Vector<double, FAST_FK_NUMBER_OF_JOINTS> &values) {
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

    void set_joints(const Eigen::Vector<double, FAST_FK_NUMBER_OF_JOINTS> &sin_values,
                    const Eigen::Vector<double, FAST_FK_NUMBER_OF_JOINTS> &cos_values) {
      for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
        joint_data[ind][0] = sin_values[ind];
        joint_data[ind][1] = cos_values[ind];
      }
    }


    double get_joint(size_t ind) {
      return asin(joint_data[ind][0]);
    }

    void get_joints(double *values) {
      for (auto ind = 0; ind < FAST_FK_NUMBER_OF_JOINTS; ++ind) {
        values[ind] = asin(joint_data[ind][0]);
      }
    }


    void get_frame(size_t index, Eigen::Matrix<double, 4, 4> &transform) {
      transform(0, 3) = joint_data[index][2];
      transform(1, 3) = joint_data[index][3];
      transform(2, 3) = joint_data[index][4];

      transform(0, 0) = joint_data[index][5];
      transform(0, 1) = joint_data[index][6];
      transform(0, 2) = joint_data[index][7];

      transform(1, 0) = joint_data[index][8];
      transform(1, 1) = joint_data[index][9];
      transform(1, 2) = joint_data[index][10];

      transform(2, 0) = joint_data[index][11];
      transform(2, 1) = joint_data[index][12];
      transform(2, 2) = joint_data[index][13];

      transform(3, 3) = 1.0;
    }
  };


  void forward_kinematics(JointData &input_data) {
    internal::forward_kinematics_internal(input_data.joint_data.data()->data());
  }

}

#endif //FAST_FORWARD_KINEMATICS_HPP
