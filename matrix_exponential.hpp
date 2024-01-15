#include <Eigen/Core>
#include "immintrin.h"

enum OP {
  ROT,
  TRANS
};


struct Data {
  std::array<Eigen::Vector<double, 4>, 12> data_; // TODO remove later
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
forward_kinematics_internal(const Eigen::Matrix<double, sizeof...(T), 7> &angles,
                            std::array<std::array<Eigen::Vector<double, 4>, 4>, sizeof...(T)> &sum_matrix_out);

template<OP ...T>
inline void forward_kinematics(const Eigen::Matrix<double, sizeof...(T), 7> &angles,
                               std::array<std::array<Eigen::Vector<double, 4>, 4>, sizeof...(T)> &sum_matrix_out);
