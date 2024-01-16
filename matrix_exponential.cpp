#include <Eigen/Core>
#include "matrix_exponential.hpp"


// [theta axis_x axis_y axis_z offset_x offset_y offset_z ]

template<OP ...T>
void
forward_kinematics_internal(std::array<double, 7 * sizeof...(T)> &input_data) {
  size_t ind = 0;

  ([&input_data, &ind]() {
    if constexpr (T == OP::ROT) {

      // t w1 w2 w3 px py pz
      constexpr int size = 7;

      if (ind > 0) {
        double &t = input_data[ind * size + 0];
        input_data[ind * size + 1] += input_data[(ind - 1) * size + 1] * t;
        input_data[ind * size + 2] += input_data[(ind - 1) * size + 2] * t;
        input_data[ind * size + 3] += input_data[(ind - 1) * size + 3] * t;
      }

      double &t = input_data[ind * size + 0];
      double &w1 = input_data[ind * size + 1];
      double &w2 = input_data[ind * size + 2];
      double &w3 = input_data[ind * size + 3];

      double &px = input_data[ind * size + 4];
      double &py = input_data[ind * size + 5];
      double &pz = input_data[ind * size + 6];
      double sin_t = sin(t);
      double cos_t_m1 = sqrt(1.0 - sin_t * sin_t) - 1.0;

      input_data[(ind+1) * size + 4] = px * (cos_t_m1 * (w2 * w2 + w3 * w3) + 1) - py * (sin_t * w3 + w1 * w2 * cos_t_m1) +
                                   pz * (sin_t * w2 - w1 * w3 * cos_t_m1);
      input_data[(ind+1) * size + 5] = px * (sin_t * w3 - w1 * w2 * cos_t_m1) + py * (cos_t_m1 * (w1 * w1 + w3 * w3) + 1) -
                                   pz * (sin_t * w1 + w2 * w3 * cos_t_m1);
      input_data[(ind+1) * size + 6] = -px * (sin_t * w2 + w1 * w3 * cos_t_m1) + py * (sin_t * w1 - w2 * w3 * cos_t_m1) +
                                   pz * (cos_t_m1 * (w1 * w1 + w2 * w2) + 1);


    } else if constexpr (T == OP::TRANS) {
//      sum_matrix_out[ind][3].array() = sum_matrix_out[ind][3].array() * 1.000001 - .7;
    }
    ind += 1;
  }(), ...);

}

//template<>
//void
//forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
//    std::array<double, 7 * 37> &input_data) {
//  forward_kinematics_internal<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
//      input_data);
//
//}


template<>
void
forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
    std::array<double, 7 * 7> &input_data) {
  forward_kinematics_internal<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(input_data);

}
//
//template<>
//void
//forward_kinematics<OP::ROT>(
//    const Eigen::Matrix<double, 1, 7> &angles,
//    std::array<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>, 1> &sum_matrix_out) {
//  forward_kinematics_internal<OP::ROT>(
//      angles, sum_matrix_out);
//
//}
//
//
