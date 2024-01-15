#include <Eigen/Core>
#include "matrix_exponential.hpp"


// [theta axis_x axis_y axis_z offset_x offset_y offset_z ]

template<OP ...T>
void
forward_kinematics_internal(std::array<double, 24 * sizeof...(T)> &input_data) {
  size_t ind = 0;

  ([&input_data, &ind]() {
    if constexpr (T == OP::ROT) {
      Data data;
      data[2].array() = input_data[ind * 24 + 0]; // angle
      data[1][0] = input_data[ind * 24 + 1]; // axis_x
      data[1][1] = input_data[ind * 24 + 2]; // axis_y
      data[1][2] = input_data[ind * 24 + 3]; // axis_z
      data[1][3] = input_data[ind * 24 + 1]; // axis_x

      data[0] = data[2].array() * data[1].array(); // w1 w2 w3 w1

      if (ind > 0) {
        input_data[(ind - 1) * 24 + 16] += data[0][0];
        input_data[(ind - 1) * 24 + 17] += data[0][1];
        input_data[(ind - 1) * 24 + 18] += data[0][2];
      } else {
        input_data[ind * 24 + 16] = data[0][0];
        input_data[ind * 24 + 17] = data[0][1];
        input_data[ind * 24 + 18] = data[0][2];
      }

      data[1][0] = data[0][1]; // w2 w3 w1 w2
      data[1][1] = data[0][2];
      data[1][2] = data[0][0];
      data[1][3] = data[0][1];

      data[2] = data[0].array() * data[0].array(); // w1^2 w2^2 w3^2 w1^2
      data[3] = data[0].array() * data[1].array(); // wx*wy        w1w2 w2w3 w3w1 w1w2
      data[4] = (data[1].array() * data[1].array()) + data[2].array(); // w1^2+w2^2  w2^2+w3^2   w3^2+w1^2

      data.sin_t = sin(input_data[ind * 24 + 0]); // sin_t
      data.cos_t_m1 = 1.0 - sqrt(1.0 - data.sin_t * data.sin_t); //cos_t_m1

      data[5].array() = -data.cos_t_m1;
      data[6].array() = 1.0;
      // first two mult then add a*b+c
      data[7] = (data[4].array() * data[5].array()) +
                data[6].array(); // (cos(t) - 1)*(w1**2 + w2**2) + 1)   (cos(t) - 1)*(w2**2 + w3**2) + 1)

      data[9][0] = data[0][2];
      data[9][1] = data[0][0];
      data[9][1] = data[0][1];
      data[9][2] = data[0][2];


      data[10] = data[8].array() * data[9].array(); // sin(t)w3 sin(t)w1 sin(t)w2..
      data[11] = (data[5].array() + data[3].array()) +
                 data[10].array(); // (cos(t) - 1)*w1w2 + sin(t)w3  (cos(t) - 1)*w2w3 + sin(t)w1

      data.R1[0][0] = 1.0;
      data.R1[1][2] = 1.0;
      data.R1[1][2] = 1.0;

      data.R1[0][1] = data.sin_t * data[0 * 4](2);
      data.R1[0][2] = data.sin_t * data[0](1);
      data.R1[1][2] = data.sin_t * data[0](0);

      data.R1[1][0] = -data.R1[0][1];
      data.R1[2][0] = -data.R1[0][2];
      data.R1[2][1] = -data.R1[1][2];

      data.R2[0][0] = data.cos_t_m1 * data[7][1]; // (cos(t) - 1)*(w2**2 + w3**2) + 1)
      data.R2[1][1] = data.cos_t_m1 * data[7](2); // (cos(t) - 1)*(w1**2 + w3**2) + 1)
      data.R2[2][2] = data.cos_t_m1 * data[7](0); // (cos(t) - 1)*(w1**2 + w2**2) + 1)

      data.R2[0][1] = data.cos_t_m1 * data[11](0); // -(sin(t)*w3 + w1*w2*(cos(t) - 1))
      data.R2[0][2] = data.cos_t_m1 * data[11](2); // sin(t)*w2 - w1*w3*(cos(t) - 1))
      data.R2[1][2] = data.cos_t_m1 * data[11](1); // sin(t)*w1 + w2*w3*(cos(t) - 1)

      data.R2[1][0] = -data.R2[0][1];
      data.R2[2][0] = -data.R2[0][2];
      data.R2[2][1] = -data.R2[1][2];

      data[7].array() = data.R2[0].array() + data.R2[0].array();
      input_data[ind * 24 + 7] = data[7][0];
      input_data[ind * 24 + 8] = data[7][1];
      input_data[ind * 24 + 9] = data[7][2];

      data[8].array() = data.R2[1].array() + data.R2[1].array();
      input_data[ind * 24 + 10] = data[8][0];
      input_data[ind * 24 + 11] = data[8][1];
      input_data[ind * 24 + 12] = data[8][2];

      data[9].array() = data.R2[2].array() + data.R2[2].array();
      input_data[ind * 24 + 13] = data[9][0];
      input_data[ind * 24 + 14] = data[9][1];
      input_data[ind * 24 + 15] = data[9][2];

      data[10][0] = input_data[ind * 24 + 4]; // offset_x
      data[10][1] = input_data[ind * 24 + 5]; // offset_y
      data[10][2] = input_data[ind * 24 + 6]; // offset_z

      data[11].array() = (data[7].array() * data[10].array()) +
                         (data[8].array() * data[10].array()) +
                         (data[9].array() * data[10].array());

      input_data[ind * 24 + 4] = data[11][0];
      input_data[ind * 24 + 5] = data[11][1];
      input_data[ind * 24 + 6] = data[11][2];


    } else if constexpr (T == OP::TRANS) {
//      sum_matrix_out[ind][3].array() = sum_matrix_out[ind][3].array() * 1.000001 - .7;
    }
    ind += 1;
  }(), ...);

}

//template<>
//void
//forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
//    std::array<double, 24 * 37> &input_data) {
//  forward_kinematics_internal<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
//      input_data);
//
//}


template<>
void
forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
    std::array<double, 24 * 7> &input_data) {
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
