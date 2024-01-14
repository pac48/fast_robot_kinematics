#include <Eigen/Core>
#include "immintrin.h"

enum OP {
  ROT,
  TRANS
};


struct Data {
  std::array<__m256d, 12> data_; // TODO remove later
  Eigen::Matrix<double, 3, 3> R1;
  Eigen::Matrix<double, 3, 3> R2;

  __m256d *data() {
    return data_.data();
  }

  __m256d &operator[](size_t ind) {
    return data_[ind];
  }


};

template<OP ...T>
inline void
forward_kinematics_internal(const Eigen::Vector<double, sizeof...(T)> &angles,
                            std::array<Eigen::Matrix<double, 3, 3>, sizeof...(T)> &sum_matrix_out);

template<OP ...T>
inline void forward_kinematics(const Eigen::Vector<double, sizeof...(T)> &angles,
                               std::array<Eigen::Matrix<double, 3, 3>, sizeof...(T)> &sum_matrix_out);

//template<>
//void
//forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
//    const Eigen::Vector<double, 19> &angles,
//    std::array<Eigen::Matrix<double, 3, 3>, 19> &sum_matrix_out);