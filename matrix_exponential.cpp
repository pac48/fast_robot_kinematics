#include "iostream"
#include <Eigen/Core>
#include "immintrin.h"
#include "matrix_exponential.hpp"


template<OP ...T>
inline void
forward_kinematics_internal(const Eigen::Vector<double, sizeof...(T)> &angles,
                            std::array<Eigen::Matrix<double, 3, 3>, sizeof...(T)> &sum_matrix_out) {
  size_t ind = 0;
  Eigen::Vector<double, sizeof...(T)> sin_t = angles.array().sin();
  Eigen::Vector<double, sizeof...(T)> cos_t_m1 = sqrt(1.0 - sin_t.array() * sin_t.array()) - 1.0;

  ([&sum_matrix_out, &ind, &cos_t_m1, &sin_t]() {
    if constexpr (T == OP::ROT) {

      double w1 = .1f;
      double w2 = .2f;
      double w3 = .3f;

//      double cos_t_m1 = cos(angles[ind]) - 1;
//      double sin_t = sqrt(1 - (cos_t_m1 + 1) * (cos_t_m1 + 1));

      Data data;
      std::array<double, 8> w = {w1, w2, w3, w1, w2, w3, w1, w2};
      memcpy(data.data(), w.data(), sizeof(w));
      auto data_double = (std::array<double, 32 * 4> *) &data;

      // data[0]  w1 w2 w3 w1
      // data[1] w2 w3 w1 w2
      data[2] = _mm256_mul_pd(data[0], data[0]); // w1^2 w2^2 w3^2 w1^2
      data[3] = _mm256_mul_pd(data[0], data[1]); // wx*wy        w1w2 w2w3 w3w1 w1w2
      data[4] = _mm256_fmadd_pd(data[1], data[1], data[2]); // w1^2+w2^2  w2^2+w3^2   w3^2+w1^2


      std::fill((double *) &data[5], (double *) &data[6], cos_t_m1[ind]); // (cos(t) - 1)
      std::fill((double *) &data[6], (double *) &data[7], 1.0);
      // first two mult then add a*b+c
      data[7] = _mm256_fmadd_pd(data[4], data[5],
                                data[6]); // (cos(t) - 1)*(w1**2 + w2**2) + 1)   (cos(t) - 1)*(w2**2 + w3**2) + 1)
      std::fill((double *) &data[8], (double *) &data[9], sin_t[ind]);

      memcpy(&data[9], &data_double->data()[0 * 4 + 2], sizeof(__m256d)); // w3 w1 w2 w3
      data[10] = _mm256_mul_pd(data[8], data[9]); // sin(t)w3 sin(t)w1 sin(t)w2..
      data[11] = _mm256_fmadd_pd(data[5], data[3],
                                 data[10]); // (cos(t) - 1)*w1w2 + sin(t)w3  (cos(t) - 1)*w2w3 + sin(t)w1


      data.R1 = Eigen::Matrix3d::Ones();
      data.R1(0, 1) = data_double->data()[0 * 4 + 2];
      data.R1(0, 2) = data_double->data()[0 * 4 + 1];
      data.R1(1, 2) = data_double->data()[0 * 4 + 0];

      data.R1(1, 0) = -data.R1(0, 1);
      data.R1(2, 0) = -data.R1(0, 2);
      data.R1(2, 1) = -data.R1(1, 2);

      data.R2 = Eigen::Matrix3d::Zero();
      data.R2(0, 0) = data_double->data()[7 * 4 + 1]; // (cos(t) - 1)*(w2**2 + w3**2) + 1)
      data.R2(1, 1) = data_double->data()[7 * 4 + 2]; // (cos(t) - 1)*(w1**2 + w3**2) + 1)
      data.R2(2, 2) = data_double->data()[7 * 4 + 0]; // (cos(t) - 1)*(w1**2 + w2**2) + 1)

      data.R2(0, 1) = -data_double->data()[11 * 4 + 0]; // -(sin(t)*w3 + w1*w2*(cos(t) - 1))
      data.R2(0, 2) = data_double->data()[11 * 4 + 2]; // sin(t)*w2 - w1*w3*(cos(t) - 1))
      data.R2(1, 2) = -data_double->data()[11 * 4 + 1]; // sin(t)*w1 + w2*w3*(cos(t) - 1)

      data.R2(1, 0) = -data.R2(0, 1);
      data.R2(2, 0) = -data.R2(0, 2);
      data.R2(2, 1) = -data.R2(1, 2);

      for (auto i = 0; i < 3; i++) {
        for (auto j = 0; j < 3; j++) {
          sum_matrix_out[ind](i, j) = -sin_t[ind] * data.R1(i, j) + (-cos_t_m1[ind]) * data.R2(i, j);
        }
      }

    }
    if constexpr (T == OP::TRANS) {
      sum_matrix_out[ind].array() = sum_matrix_out[ind].array() * 1.000001 - .7;
    }
    ind += 1;
  }(), ...);

}

template<>
void
forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
    const Eigen::Vector<double, 19> &angles,
    std::array<Eigen::Matrix<double, 3, 3>, 19> &sum_matrix_out) {

  forward_kinematics_internal<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
      angles, sum_matrix_out);

}

template<>
void
forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
    const Eigen::Vector<double, 7> &angles,
    std::array<Eigen::Matrix<double, 3, 3>, 7> &sum_matrix_out) {

  forward_kinematics_internal<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
      angles, sum_matrix_out);

}


