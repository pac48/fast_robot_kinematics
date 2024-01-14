#include "chrono"
#include "iostream"
#include <Eigen/Core>
#include "immintrin.h"

enum OP {
  ROT,
  TRANS
};


struct Data {
  alignas(64) std::array<__m256d, 12> data_ = {0}; // TODO remove later
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
forward_kinematics(const Eigen::Vector<double, sizeof...(T)> &angles,
                   std::array<Eigen::Matrix<double, 3, 3>, sizeof...(T)> &sum_matrix_out) {

//  Eigen::Vector<double, sizeof...(T)> cos_t_m1 = angles.array().cos() - 1;
//  const Eigen::Vector<double, sizeof...(T)> sin_t = sqrt(1 - cos_t_m1.array() * cos_t_m1.array());
//  cos_t_m1.array() -= 1.0;


  size_t ind = 0;
  ([&sum_matrix_out, &ind, &angles]() {
    if constexpr (T == OP::ROT) {

      double w1 = .1f * ind;
      double w2 = .2f * ind;
      double w3 = .3f * ind;
      double cos_t_m1 = cos(angles[ind]) - 1;
      double sin_t = sqrt(1 - (cos_t_m1 + 1) * (cos_t_m1 + 1));

      Data data;
      std::array<double, 8> w = {w1, w2, w3, w1, w2, w3, w1, w2};
      memcpy(data.data(), w.data(), sizeof(w));
      auto data_double = (std::array<double, 32 * 4> *) &data;

      // data[0]  w1 w2 w3 w1
      // data[1] w2 w3 w1 w2
      data[2] = _mm256_mul_pd(data[0], data[0]); // w1^2 w2^2 w3^2 w1^2
      data[3] = _mm256_mul_pd(data[0], data[1]); // wx*wy        w1w2 w2w3 w3w1 w1w2
      data[4] = _mm256_fmadd_pd(data[1], data[1], data[2]); // w1^2+w2^2  w2^2+w3^2   w3^2+w1^2


      std::fill((double *) &data[5], (double *) &data[6], cos_t_m1); // (cos(t) - 1)
      std::fill((double *) &data[6], (double *) &data[7], 1.0);
      // first two mult then add a*b+c
      data[7] = _mm256_fmadd_pd(data[4], data[5],
                                data[6]); // (cos(t) - 1)*(w1**2 + w2**2) + 1)   (cos(t) - 1)*(w2**2 + w3**2) + 1)
      std::fill((double *) &data[8], (double *) &data[9], sin_t);

      memcpy(&data[9], &data_double->data()[0 * 4 + 2], sizeof(__m256d)); // w3 w1 w2 w3
      data[10] = _mm256_mul_pd(data[8], data[9]); // sin(t)w3 sin(t)w1 sin(t)w2..
      data[11] = _mm256_fmadd_pd(data[5], data[3],
                                 data[10]); // (cos(t) - 1)*w1w2 + sin(t)w3  (cos(t) - 1)*w2w3 + sin(t)w1


      data.R1 = Eigen::Matrix3d::Zero();
      data.R1(0, 1) = -sin_t * data_double->data()[0 * 4 + 2];
      data.R1(0, 2) = -sin_t * data_double->data()[0 * 4 + 1];
      data.R1(1, 2) = -sin_t * data_double->data()[0 * 4 + 0];

      data.R1(1, 0) = -data.R1(0, 1);
      data.R1(2, 0) = -data.R1(0, 2);
      data.R1(2, 1) = -data.R1(1, 2);

      data.R2 = Eigen::Matrix3d::Zero();
      data.R2(0, 0) = (-cos_t_m1) * data_double->data()[7 * 4 + 1]; // (cos(t) - 1)*(w2**2 + w3**2) + 1)
      data.R2(1, 1) = (-cos_t_m1) * data_double->data()[7 * 4 + 2]; // (cos(t) - 1)*(w1**2 + w3**2) + 1)
      data.R2(2, 2) = (-cos_t_m1) * data_double->data()[7 * 4 + 0]; // (cos(t) - 1)*(w1**2 + w2**2) + 1)

      data.R2(0, 0) += 1.0;
      data.R2(1, 1) += 1.0;
      data.R2(2, 2) += 1.0;

      data.R2(0, 1) = -data_double->data()[11 * 4 + 0]; // -(sin(t)*w3 + w1*w2*(cos(t) - 1))
      data.R2(0, 2) = data_double->data()[11 * 4 + 2]; // sin(t)*w2 - w1*w3*(cos(t) - 1))
      data.R2(1, 2) = -data_double->data()[11 * 4 + 1]; // sin(t)*w1 + w2*w3*(cos(t) - 1)

      data.R2(1, 0) = -data.R2(0, 1);
      data.R2(2, 0) = -data.R2(0, 2);
      data.R2(2, 1) = -data.R2(1, 2);


//      sum_matrix_out[ind].array() = data.R1.array() + data.R2.array();

      for (auto i=0;i < 9; i++){
        sum_matrix_out[ind].data()[i] = data.R1.data()[i] + data.R2.data()[i];
      }


//      sum_matrix_out[ind](0, 0) = data.R1(0, 0) + data.R2(0, 0);
//      sum_matrix_out[ind](0, 1) = data.R1(0, 1) + data.R2(0, 1);
//      sum_matrix_out[ind](0, 2) = data.R1(0, 2) + data.R2(0, 2);
//      sum_matrix_out[ind](1, 0) = data.R1(1, 0) + data.R2(1, 0);
//      sum_matrix_out[ind].data()[4] = data.R1.data()[4] + data.R2.data()[4];
//      sum_matrix_out[ind](1, 2) = data.R1(1, 2) + data.R2(1, 2);
//      sum_matrix_out[ind](2, 0) = data.R1(2, 0) + data.R2(2, 0);
//      sum_matrix_out[ind](2, 1) = data.R1(2, 1) + data.R2(2, 1);
//      sum_matrix_out[ind](2, 3) = data.R1(2, 3) + data.R2(2, 3);


//      double val = data.R2(2, 1);
//      memcpy(&val, &data.data()[8], sizeof(double));
//      printf("%f here!! \n", val);
//      sum_matrix_out[ind].array() = val;// + .1;

    }
    if constexpr (T == OP::TRANS) {
      sum_matrix_out[ind].array() = sum_matrix_out[ind].array() * 1.000001 - .7;
    }
    ind += 1;
  }(), ...);

//  return sum_matrix_out;
}


int main(int arc, char **argv) {

  // working
//  double w1 = .1f * arc;
//  double w2 = .2f * arc;
//  double w3 = .3f * arc;
//  double theta = 2.3 * arc;
//  double cos_t_m1 = cos(theta) - 1;
//  double sin_t = sqrt(1 - (cos_t_m1 + 1) * (cos_t_m1 + 1));
//
//  Data data;
//  std::array<double, 8> w = {w1, w2, w3, w1, w2, w3, w1, w2};
//  memcpy(data.data(), w.data(), sizeof(w));
//  auto data_double = (std::array<double, 32 * 4> *) &data;
//
////  data[0]  w1 w2 w3 w1
////  data[1] w2 w3 w1 w2
//  data[2] = _mm256_mul_pd(data[0], data[0]); // w1^2 w2^2 w3^2 w1^2
//  data[3] = _mm256_mul_pd(data[0], data[1]); // wx*wy        w1w2 w2w3 w3w1 w1w2
//  data[4] = _mm256_fmadd_pd(data[1], data[1], data[2]); // w1^2+w2^2  w2^2+w3^2   w3^2+w1^2
//
//
//  std::fill((double *) &data[5], (double *) &data[6], cos_t_m1); // (cos(t) - 1)
//  std::fill((double *) &data[6], (double *) &data[7], 1.0);
//  // first two mult then add a*b+c
//  data[7] = _mm256_fmadd_pd(data[4], data[5],
//                            data[6]); // (cos(t) - 1)*(w1**2 + w2**2) + 1)   (cos(t) - 1)*(w2**2 + w3**2) + 1)
//  std::fill((double *) &data[8], (double *) &data[9], sin_t);
//
//  memcpy(&data[9], &data_double->data()[0 * 4 + 2], sizeof(__m256d)); // w3 w1 w2 w3
//  data[10] = _mm256_mul_pd(data[8], data[9]); // sin(t)w3 sin(t)w1 sin(t)w2..
//  data[11] = _mm256_fmadd_pd(data[5], data[3], data[10]); // (cos(t) - 1)*w1w2 + sin(t)w3  (cos(t) - 1)*w2w3 + sin(t)w1
//
//  data.R = Eigen::Matrix3d::Zero();
//  data.R(0, 0) = data_double->data()[7 * 4 + 1]; // (cos(t) - 1)*(w2**2 + w3**2) + 1)
//  data.R(1, 1) = data_double->data()[7 * 4 + 2]; // (cos(t) - 1)*(w1**2 + w3**2) + 1)
//  data.R(2, 2) = data_double->data()[7 * 4 + 0]; // (cos(t) - 1)*(w1**2 + w2**2) + 1)
//
//  data.R(0, 1) = -data_double->data()[11 * 4 + 0]; // -(sin(t)*w3 + w1*w2*(cos(t) - 1))
//  data.R(0, 2) = data_double->data()[11 * 4 + 2]; // sin(t)*w2 - w1*w3*(cos(t) - 1))
//  data.R(1, 2) = -data_double->data()[11 * 4 + 1]; // sin(t)*w1 + w2*w3*(cos(t) - 1)
//
//  data.R(1, 0) = -data.R(0, 1);
//  data.R(2, 0) = -data.R(0, 2);
//  data.R(2, 1) = -data.R(1, 2);
//
//  int i = 0;
//  int count = 0;
//  for (const auto &tmp: *data_double) {
//    if (i == 0) {
//      printf("\n%d: %f ", count / 4, tmp);
//    } else {
//      printf("%f ", tmp);
//    }
//    i = ((i + 1) % 4);
//    count++;
//  }
//
//  printf("%f", data.R(0, 0));

//  return 0;

  constexpr int iterations = 128 * 128 * 128*10;
  std::array<Eigen::Matrix<double, 3, 3>, 19> sum_matrix;
  Eigen::Vector<double, 19> angles;
  angles
      << -.01, .02, -.03, .04, -.05, .06, -.07, .08, -.09, .010, -.011, .012, -.013, .014, -.015, .016, -.017, .018, -.019;
  angles *= arc;

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; i++) {
//    angles.array() += .0001*(.5 - double (rand() % 100)/100);

    forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
        angles.array(), sum_matrix);
//    angles.array() += .1 * (.5 - (i % 2)) * sum_matrix[1](0, 0);
    angles.array() += 1E-4*sum_matrix[17].sum();
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

  for (auto &val: angles) {
    std::cout << "final value: " << val << std::endl;
  }

  std::cout << "Time taken by function: " << (double) duration.count() << " nanoseconds" << std::endl;
  std::cout << "Average: " << ((double) duration.count()) / (iterations) << " nanoseconds"
            << std::endl;

  return 0;
}
