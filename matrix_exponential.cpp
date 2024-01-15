#include <Eigen/Core>
#include "matrix_exponential.hpp"


// [theta axis_x axis_y axis_z offset_x offset_y offset_z ]

template<OP ...T>
void
forward_kinematics_internal(const Eigen::Matrix<double, sizeof...(T), 7> &angles,
                            std::array<std::array<Eigen::Vector<double, 4>, 4>, sizeof...(T)> &sum_matrix_out) {
  size_t ind = 0;
  const Eigen::Vector<double, sizeof...(T)> sin_t = angles.col(0).array().sin();
  const Eigen::Vector<double, sizeof...(T)> cos_t_m1 = (1.0 - sin_t.array() * sin_t.array()).sqrt() - 1.0;
//  Eigen::Matrix<double, sizeof...(T) + 1, 4> W = Eigen::Matrix<double, sizeof...(T) + 1, 4>::Zero();
//  W << 0, 0, 0, 0;
//  for (auto i = 0; i < sizeof...(T); i++) {
//    W.block(i + 1, 0, 1, 3).array() = W.block(i, 0, 1, 3).array() + angles.block(i, 1, 1, 3).array() * angles(i, 0);
//  }
  Eigen::Vector<double, 4> W = Eigen::Vector<double, 4>::Zero();

  ([&sum_matrix_out, &ind, &cos_t_m1, &sin_t, &W, &angles]() {
    if constexpr (T == OP::ROT) {
//      auto row = angles.row(ind);
//      W.array() += row(0)*row.block(0, 1, 1, 3).array();
//      W[0] += angles(ind, 1)*angles(ind, 0);
//      W[1] += angles(ind, 2)*angles(ind, 0);
//      W[2] += angles(ind, 3)*angles(ind, 0);

      Data data;
      data[0].array() = angles(ind, 0);
      data[1][0] = angles(ind, 1);
      data[1][1] = angles(ind, 2);
      data[1][2] = angles(ind, 3);
//      data[1][3] = angles(ind, 4);

//      memcpy(data[1].data(), &angles(ind, 1), sizeof(double) * 4);

      W.array() += data[0].array() * data[1].array();// angles.block(ind, 1, 1, 3).array();

      data[0] = W.array(); // w1 w2 w3 w4

      data[1][0] = data[0][1]; // w2 w3 w1 w2
      data[1][1] = data[0][2];
      data[1][2] = data[0][0];
      data[1][3] = data[0][1];

//      data[1] << data[0][1], data[0][2], data[0][0], data[0][1];
//      memcpy(data[1].data(), data[0].data() + 1, sizeof(double) * 4);

      data[2] = data[0].array() * data[0].array(); // w1^2 w2^2 w3^2 w1^2
      data[3] = data[0].array() * data[1].array(); // wx*wy        w1w2 w2w3 w3w1 w1w2
      data[4] = (data[1].array() * data[1].array()) + data[2].array(); // w1^2+w2^2  w2^2+w3^2   w3^2+w1^2

      data[5].array() = cos_t_m1[ind];
      data[6].array() = 1.0;
      // first two mult then add a*b+c
      data[7] = (data[4].array() * data[5].array()) +
                data[6].array(); // (cos(t) - 1)*(w1**2 + w2**2) + 1)   (cos(t) - 1)*(w2**2 + w3**2) + 1)
      data[8].array() = sin_t[ind];

      data[9] << data[0][2], data[0][0], data[0][1], data[0][2];
      data[10] = data[8].array() * data[9].array(); // sin(t)w3 sin(t)w1 sin(t)w2..
      data[11] = (data[5].array() + data[3].array()) +
                 data[10].array(); // (cos(t) - 1)*w1w2 + sin(t)w3  (cos(t) - 1)*w2w3 + sin(t)w1


//      data.R1 = Eigen::Matrix3d::Ones();
      data.R1[0][0] = 1.0;
      data.R1[1][2] = 1.0;
      data.R1[1][2] = 1.0;

//      data[0].array() *= -sin_t[ind];

      data.R1[0][1] = sin_t[ind]*data[0 * 4](2);
      data.R1[0][2] = sin_t[ind]*data[0](1);
      data.R1[1][2] = sin_t[ind]*data[0](0);

      data.R1[1][0] = -data.R1[0][1];
      data.R1[2][0] = -data.R1[0][2];
      data.R1[2][1] = -data.R1[1][2];

//      data.R2 = Eigen::Matrix3d::Zero();
//      data[7].array() *= -cos_t_m1[ind];
//      data[11].array() *= -cos_t_m1[ind];

      data.R2[0][0] = -cos_t_m1[ind]*data[7][1]; // (cos(t) - 1)*(w2**2 + w3**2) + 1)
      data.R2[1][1] = -cos_t_m1[ind]*data[7](2); // (cos(t) - 1)*(w1**2 + w3**2) + 1)
      data.R2[2][2] = -cos_t_m1[ind]*data[7](0); // (cos(t) - 1)*(w1**2 + w2**2) + 1)

      data.R2[0][1] = -cos_t_m1[ind]*data[11](0); // -(sin(t)*w3 + w1*w2*(cos(t) - 1))
      data.R2[0][2] = -cos_t_m1[ind]*data[11](2); // sin(t)*w2 - w1*w3*(cos(t) - 1))
      data.R2[1][2] = -cos_t_m1[ind]*data[11](1); // sin(t)*w1 + w2*w3*(cos(t) - 1)

      data.R2[1][0] = -data.R2[0][1];
      data.R2[2][0] = -data.R2[0][2];
      data.R2[2][1] = -data.R2[1][2];


//      for (auto j = 0; j < 3; j++) {
//        for (auto i = 0; i < 3; i++) {
//          sum_matrix_out[ind](i, j) = -sin_t[ind] * data.R1(i, j) + (-cos_t_m1[ind]) * data.R2(i, j);
//        }
//      }

//      memcpy(data[7].data(), data.R1.data(), sizeof(double) * 3);
//      memcpy(data[8].data(), data.R1.data() + 3, sizeof(double) * 3);
//      memcpy(data[9].data(), data.R1.data() + 6, sizeof(double) * 3);
//
//      memcpy(data[4].data(), data.R2.data(), sizeof(double) * 3);
//      memcpy(data[5].data(), data.R2.data() + 3, sizeof(double) * 3);
//      memcpy(data[6].data(), data.R2.data() + 6, sizeof(double) * 3);

      sum_matrix_out[ind][0].array() = data.R2[0].array() + data.R2[0].array();
      sum_matrix_out[ind][1].array() = data.R2[1].array() + data.R2[1].array();
      sum_matrix_out[ind][2].array() = data.R2[2].array() + data.R2[2].array();

//      data[10].array() = data[7].array() + data[4].array();
//      memcpy(sum_matrix_out[ind].data(), data[10].data(), sizeof(double) * 3);
//
//      data[10].array() = data[8].array() + data[5].array();
//      memcpy(sum_matrix_out[ind].data() + 4, data[10].data(), sizeof(double) * 3);
//
//      data[10].array() = data[9].array() + data[6].array();
//      memcpy(sum_matrix_out[ind].data() + 8, data[10].data(), sizeof(double) * 3);


//      sum_matrix_out[ind].row(3) = sum_matrix_out[ind].block(0, 0, 3, 3) * angles.row(ind).block(0, 4, 1, 3);

//      sum_matrix_out[ind](3, 0) = sum_matrix_out[ind](0, 0)*angles(ind, 4) + sum_matrix_out[ind](0, 1)*angles(ind, 5)+ sum_matrix_out[ind](0, 2)*angles(ind, 6);
//      sum_matrix_out[ind](3, 1) = sum_matrix_out[ind](1, 0)*angles(ind, 4) + sum_matrix_out[ind](1, 1)*angles(ind, 5)+ sum_matrix_out[ind](1, 2)*angles(ind, 6);
//      sum_matrix_out[ind](3, 2) = sum_matrix_out[ind](2, 0)*angles(ind, 4) + sum_matrix_out[ind](2, 1)*angles(ind, 5)+ sum_matrix_out[ind](2, 2)*angles(ind, 6);

      // x


      data[11].array() = angles(ind, 4); // offsets
      sum_matrix_out[ind][3].array() = 0;
      sum_matrix_out[ind][3].array() += sum_matrix_out[ind][0].array()*data[11].array();
      sum_matrix_out[ind][3].array() += sum_matrix_out[ind][1].array()*data[11].array();
      sum_matrix_out[ind][3].array() += sum_matrix_out[ind][2].array()*data[11].array();

//
//      memcpy(data[10].data(), sum_matrix_out[ind].data(), sizeof(double) * 4);
//      data[11].array() += data[9].array() + data[10].array();
//
//      data[9].array() = angles(ind, 5);
//      memcpy(data[10].data(), sum_matrix_out[ind].data() + 4, sizeof(double) * 4);
//      data[11].array() += data[9].array() + data[10].array();
//
//      data[9].array() = angles(ind, 6);
//      memcpy(data[10].data(), sum_matrix_out[ind].data() + 8, sizeof(double) * 4);
//      data[11].array() += data[9].array() + data[10].array();


//      sum_matrix_out[ind].col(3).array() += sum_matrix_out[ind].col(0).array() * data[11].array();
//
//      data[11].array() = angles(ind, 5);
//      sum_matrix_out[ind].col(3).array() += sum_matrix_out[ind].col(1).array() * data[11].array();
//
//      data[11].array() = angles(ind, 6);
//      sum_matrix_out[ind].col(3).array() += sum_matrix_out[ind].col(2).array() * data[11].array();


//      sum_matrix_out[ind](1, 3) = sum_matrix_out[ind](1, 0) * angles(ind, 4);
//      sum_matrix_out[ind](2, 3) = sum_matrix_out[ind](2, 0) * angles(ind, 4);



//      Eigen::Vector3d vec;
//      vec << cos_t_m1[0], cos_t_m1[1], cos_t_m1[2];
//      sum_matrix_out[ind].row(4) = sum_matrix_out[ind].block(0, 0, 3, 3) * angles.row(ind).block(0, 4, 1, 3);
//      sum_matrix_out[ind](0,0) = vec[0];
//      sum_matrix_out[ind](0,1) = vec[1];
//      sum_matrix_out[ind](0,2) = vec[2];

    } else if constexpr (T == OP::TRANS) {
      sum_matrix_out[ind][3].array() = sum_matrix_out[ind][3].array() * 1.000001 - .7;
    }
    ind += 1;
  }(), ...);

}

template<>
void
forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
    const Eigen::Matrix<double, 19, 7> &angles,
    std::array<std::array<Eigen::Vector<double, 4>, 4>, 19> &sum_matrix_out) {

  forward_kinematics_internal<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
      angles, sum_matrix_out);

}

template<>
void
forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
    const Eigen::Matrix<double, 7, 7> &angles,
    std::array<std::array<Eigen::Vector<double, 4>, 4>, 7> &sum_matrix_out) {

  forward_kinematics_internal<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
      angles, sum_matrix_out);

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
