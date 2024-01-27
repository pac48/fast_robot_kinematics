#include <Eigen/Core>
#include "matrix_exponential.hpp"


constexpr std::array<double, 7 * 9> R_all = {
    0.64899115, 0.82075596, 0.65632031, 0.52333051, 0.0405908,
    0.89556638, 0.74368786, 0.89381274, 0.33964751, 0.53515741,
    0.93426882, 0.50967539, 0.47704657, 0.19292437, 0.10204081,
    0.30090257, 0.42999095, 0.87449618, 0.30207915, 0.50739501,
    0.32769396, 0.39646723, 0.96918674, 0.72854786, 0.53141685,
    0.73213793, 0.03204814, 0.7763471, 0.77061773, 0.75143078,
    0.2124764, 0.26393017, 0.53043246, 0.03268181, 0.42973274,
    0.20423035, 0.61280869, 0.50943287, 0.60075293, 0.59731078,
    0.36964367, 0.75134004, 0.33956527, 0.3895274, 0.98645527,
    0.99857242, 0.02781383, 0.07846183, 0.77213754, 0.03395741,
    0.3192952, 0.98976506, 0.59917512, 0.21160893, 0.96988492,
    0.53811592, 0.8298471, 0.11268434, 0.99029304, 0.89830518,
    0.74575603, 0.30309202, 0.81797937
};

constexpr std::array<double, 7 * 3> offset_all = {
    0.71905754, 0.38351185, 0.12772781, 0.21024537, 0.93051326,
    0.32022412, 0.33151422, 0.10613476, 0.71060096, 0.81162218,
    0.23844306, 0.11828732, 0.75801453, 0.42562381, 0.44630478,
    0.18564595, 0.34140355, 0.11053668, 0.60855285, 0.38914153,
    0.27247183
};


template<OP ...T>
void
forward_kinematics_internal(std::array<double, 17 * sizeof...(T)> &input_data) {
  size_t ind = 0;
  ([&input_data, &ind]() {
    if constexpr (T == OP::ROT) {

      // sin(t) cos(t)  R11, R12, R13... px py pz
      constexpr int size = 2 + 3 + 9 + 3; // 17

      const double &sin_t = input_data[ind * size + 0];
      const double &cos_t = input_data[ind * size + 1];

      // row major
      const double &R11_fixed = R_all[ind * 9 + 0];
      const double &R21_fixed = R_all[ind * 9 + 1];
      const double &R31_fixed = R_all[ind * 9 + 2];
      const double &R12_fixed = R_all[ind * 9 + 3];
      const double &R22_fixed = R_all[ind * 9 + 4];
      const double &R32_fixed = R_all[ind * 9 + 5];
      const double &R13_fixed = R_all[ind * 9 + 6];
      const double &R23_fixed = R_all[ind * 9 + 7];
      const double &R33_fixed = R_all[ind * 9 + 8];

      const double &offset_x = offset_all[ind * 3 + 0];
      const double &offset_y = offset_all[ind * 3 + 1];
      const double &offset_z = offset_all[ind * 3 + 2];

      // column major
      double &R11 = input_data[ind * size + 5];
      double &R12 = input_data[ind * size + 6];
      double &R13 = input_data[ind * size + 7];
      double &R21 = input_data[ind * size + 8];
      double &R22 = input_data[ind * size + 9];
      double &R23 = input_data[ind * size + 10];
      double &R31 = input_data[ind * size + 11];
      double &R32 = input_data[ind * size + 12];
      double &R33 = input_data[ind * size + 13];

      if (ind == 0) {
        // calculate R_fixed*R_joint (build column major)

        R11 = R11_fixed * cos_t + R12_fixed * sin_t;
        R12 = -R11_fixed * sin_t + R12_fixed * cos_t;
        R13 = R13_fixed;
        R21 = R21_fixed * cos_t + R22_fixed * sin_t;
        R22 = -R21_fixed * sin_t + R22_fixed * cos_t;
        R23 = R23_fixed;
        R31 = R31_fixed * cos_t + R32_fixed * sin_t;
        R32 = -R31_fixed * sin_t + R32_fixed * cos_t;
        R33 = R33_fixed;

//        double &tmp1 = input_data[ind * size + 14];
//        double &tmp2 = input_data[ind * size + 15];
//        double &tmp3 = input_data[ind * size + 16];

//        tmp1 = R11 * offset_x + R12 * offset_y + R13 * offset_z;
//        tmp2 = R21 * offset_x + R22 * offset_y + R23 * offset_z;
//        tmp3 = R31 * offset_x + R32 * offset_y + R33 * offset_z;

        input_data[ind * size + 2] =  R11 * offset_x + R12 * offset_y + R13 * offset_z;
        input_data[ind * size + 3] = R21 * offset_x + R22 * offset_y + R23 * offset_z;
        input_data[ind * size + 4] = R31 * offset_x + R32 * offset_y + R33 * offset_z;


      } else {
        // calculate R_cum*R_fixed*R_joint (build column major)

        // calculate R_fixed*R_joint (build row major)
        // R_tmp row major
        double &R11_tmp = input_data[ind * size + 5];
        double &R21_tmp = input_data[ind * size + 6];
        double &R31_tmp = input_data[ind * size + 7];
        double &R12_tmp = input_data[ind * size + 8];
        double &R22_tmp = input_data[ind * size + 9];
        double &R32_tmp = input_data[ind * size + 10];
        double &R13_tmp = input_data[ind * size + 11];
        double &R23_tmp = input_data[ind * size + 12];
        double &R33_tmp = input_data[ind * size + 13];

        input_data[ind * size + 5] = R11_fixed * cos_t + R12_fixed * sin_t;
        input_data[ind * size + 6] = R21_fixed * cos_t + R22_fixed * sin_t;
        input_data[ind * size + 7] = R31_fixed * cos_t + R32_fixed * sin_t;
        input_data[ind * size + 8] = -R11_fixed * sin_t + R12_fixed * cos_t;
        input_data[ind * size + 9] = -R21_fixed * sin_t + R22_fixed * cos_t;
        input_data[ind * size + 10] = -R31_fixed * sin_t + R32_fixed * cos_t;
        input_data[ind * size + 11] = R13_fixed;
        input_data[ind * size + 12] = R23_fixed;
        input_data[ind * size + 13] = R33_fixed;

        // R_cum_next current it R_cum*R_fixed*R_joint  (build column major)
        double &tmp1 = input_data[ind * size + 14];
        double &tmp2 = input_data[ind * size + 15];
        double &tmp3 = input_data[ind * size + 16];

        double &R11_old = input_data[(ind - 1) * size + 5];
        double &R12_old = input_data[(ind - 1) * size + 6];
        double &R13_old = input_data[(ind - 1) * size + 7];
        double &R21_old = input_data[(ind - 1) * size + 8];
        double &R22_old = input_data[(ind - 1) * size + 9];
        double &R23_old = input_data[(ind - 1) * size + 10];
        double &R31_old = input_data[(ind - 1) * size + 11];
        double &R32_old = input_data[(ind - 1) * size + 12];
        double &R33_old = input_data[(ind - 1) * size + 13];

        tmp1 = R11_old * R11_tmp + R12_old * R21_tmp + R13_old * R31_tmp;
        tmp2 = R21_old * R11_tmp + R22_old * R21_tmp + R23_old * R31_tmp;
        tmp3 = R31_old * R11_tmp + R32_old * R21_tmp + R33_old * R31_tmp;
        R11 = tmp1;
        R21 = tmp2;
        R31 = tmp3;

        tmp1 = R11_old * R12_tmp + R12_old * R22_tmp + R13_old * R32_tmp;
        tmp2 = R21_old * R12_tmp + R22_old * R22_tmp + R23_old * R32_tmp;
        tmp3 = R31_old * R12_tmp + R32_old * R22_tmp + R33_old * R32_tmp;
        R12 = tmp1;
        R22 = tmp2;
        R32 = tmp3;

        tmp1 = R11_old * R13_tmp + R12_old * R23_tmp + R13_old * R33_tmp;
        tmp2 = R21_old * R13_tmp + R22_old * R23_tmp + R23_old * R33_tmp;
        tmp3 = R31_old * R13_tmp + R32_old * R23_tmp + R33_old * R33_tmp;
        R13 = tmp1;
        R23 = tmp2;
        R33 = tmp3;

        double &px_old = input_data[(ind - 1) * size + 2];
        double &py_old = input_data[(ind - 1) * size + 3];
        double &pz_old = input_data[(ind - 1) * size + 4];

        tmp1 = R11 * offset_x + R12 * offset_y + R13 * offset_z + px_old;
        tmp2 = R21 * offset_x + R22 * offset_y + R23 * offset_z + py_old;
        tmp3 = R31 * offset_x + R32 * offset_y + R33 * offset_z + pz_old;

//        double &px = input_data[ind * size + 2];
//        double &py = input_data[ind * size + 3];
//        double &pz = input_data[ind * size + 4];

        input_data[ind * size + 2] = tmp1;
        input_data[ind * size + 3] = tmp2;
        input_data[ind * size + 4] = tmp3;


      }


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
    std::array<double, 7 * 17> &input_data) {
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
