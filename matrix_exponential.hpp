#include <Eigen/Core>
#include "immintrin.h"

enum OP {
  ROT,
  TRANS
};


struct Data {
  std::array<Eigen::Vector<double, 4>, 12> data_; // TODO remove later
  double sin_t;
  double cos_t_m1;
  std::array<Eigen::Vector<double, 4>, 4> R1;
  std::array<Eigen::Vector<double, 4>, 4> R2;

  Eigen::Vector4d *data() {
    return data_.data();
  }

  Eigen::Vector4d &operator[](size_t ind) {
    return data_[ind];
  }


};

template<OP ...T>
inline void
forward_kinematics_internal(std::array<double, 24 * sizeof...(T)> &input_data);

template<OP ...T>
inline void forward_kinematics(std::array<double, 24 * sizeof...(T)> &input_data);
