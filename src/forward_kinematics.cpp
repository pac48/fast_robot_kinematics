#pragma once

#include "forward_kinematics.hpp"

namespace fast_fk {
  enum OP {
    ROT,
    TRANS
  };

  // row major
  constexpr std::array<double, 6 * 9> R_all = {1.0, 4.898391345320441e-09, 0.0,
                                               -4.898391345320441e-09, 1.0, 0.0,
                                               0.0, 0.0, 1.0,
                                               0.9999999999920636, -2.79423007149925e-09, 3.984081429823977e-06,
                                               -3.984082409688579e-06, -0.0007013484622411433, 0.9999997540472005,
                                               0.0, -0.999999754055137, -0.0007013484622467095,
                                               0.9999993163649181, 5.651241854527351e-06, -0.001169289425224364,
                                               -4.425873088671482e-06, 0.9999994508798468, 0.0010479601216713834,
                                               0.001169294705420077, -0.00104795423012248, 0.9999987672701519,
                                               0.9999999072555382, -1.0253697707014652e-05, 0.00043056216350321623,
                                               7.033674472410706e-06, 0.9999720394582239, 0.0074779845070412285,
                                               -0.0004306268017444494, -0.007477980785065483, 0.9999719467884766,
                                               0.9999999999999949, -1.835833733132026e-10, -1.012782353923603e-07,
                                               1.0127840177967513e-07, 0.001812660647159262, 0.9999983571293345,
                                               0.0, -0.9999983571293396, 0.0018126606471592713,
                                               0.9999999999999998, 2.3336091799563132e-11, -2.0405379709517242e-08,
                                               -2.0405393053375827e-08, 0.0011436177325240319, -0.9999993460690269,
                                               -1.2246467991473532e-16, 0.9999993460690271, 0.001143617732524032,
  };


  constexpr std::array<double, 6 * 3> offset_all = {0.0, 0.0, 0.1625702965797758,
                                                    0.000182214465989093, 0.0, 0.0,
                                                    -0.4249817627044961, 0.0, 0.0,
                                                    -0.3921666446509172, -0.0009975307642066673, 0.1333919956383524,
                                                    2.398975523480517e-06, -0.09959821611958637, 0.0001805380634879481,
                                                    7.603964784130673e-05, 0.09950302422228456, 0.0001137934973534554,
  };


  template<OP ...T>
  inline void
  forward_kinematics_internal(double *input_data) {
    // sin(t) cos(t)  R11, R12, R13... px py pz
    constexpr int size = 2 + 3 + 9 + 3; // 17

    std::size_t ind = 0;
    ([&input_data, &ind]() {
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

      // column major
      double &R11_old = input_data[(ind - 1) * size + 5];
      double &R12_old = input_data[(ind - 1) * size + 6];
      double &R13_old = input_data[(ind - 1) * size + 7];
      double &R21_old = input_data[(ind - 1) * size + 8];
      double &R22_old = input_data[(ind - 1) * size + 9];
      double &R23_old = input_data[(ind - 1) * size + 10];
      double &R31_old = input_data[(ind - 1) * size + 11];
      double &R32_old = input_data[(ind - 1) * size + 12];
      double &R33_old = input_data[(ind - 1) * size + 13];

      if constexpr (T == OP::ROT) {
        const double &sin_t = input_data[ind * size + 0];
        const double &cos_t = input_data[ind * size + 1];

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

          input_data[ind * size + 2] = R11 * offset_x + R12 * offset_y + R13 * offset_z;
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

          R11_tmp = R11_fixed * cos_t + R12_fixed * sin_t;
          R21_tmp = R21_fixed * cos_t + R22_fixed * sin_t;
          R31_tmp = R31_fixed * cos_t + R32_fixed * sin_t;
          R12_tmp = -R11_fixed * sin_t + R12_fixed * cos_t;
          R22_tmp = -R21_fixed * sin_t + R22_fixed * cos_t;
          R32_tmp = -R31_fixed * sin_t + R32_fixed * cos_t;
          R13_tmp = R13_fixed;
          R23_tmp = R23_fixed;
          R33_tmp = R33_fixed;

          // R_cum_next current it R_cum*R_fixed*R_joint  (build column major)
          double &tmp1 = input_data[ind * size + 14];
          double &tmp2 = input_data[ind * size + 15];
          double &tmp3 = input_data[ind * size + 16];

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

          input_data[ind * size + 2] = tmp1;
          input_data[ind * size + 3] = tmp2;
          input_data[ind * size + 4] = tmp3;


        }

      } else if constexpr (T == OP::TRANS) {
        double &tmp1 = input_data[ind * size + 14];
        double &tmp2 = input_data[ind * size + 15];
        double &tmp3 = input_data[ind * size + 16];

        tmp1 = R11_old * R11_fixed + R12_old * R21_fixed + R13_old * R31_fixed;
        tmp2 = R21_old * R11_fixed + R22_old * R21_fixed + R23_old * R31_fixed;
        tmp3 = R31_old * R11_fixed + R32_old * R21_fixed + R33_old * R31_fixed;
        R11 = tmp1;
        R21 = tmp2;
        R31 = tmp3;

        tmp1 = R11_old * R12_fixed + R12_old * R22_fixed + R13_old * R32_fixed;
        tmp2 = R21_old * R12_fixed + R22_old * R22_fixed + R23_old * R32_fixed;
        tmp3 = R31_old * R12_fixed + R32_old * R22_fixed + R33_old * R32_fixed;
        R12 = tmp1;
        R22 = tmp2;
        R32 = tmp3;

        tmp1 = R11_old * R13_fixed + R12_old * R23_fixed + R13_old * R33_fixed;
        tmp2 = R21_old * R13_fixed + R22_old * R23_fixed + R23_old * R33_fixed;
        tmp3 = R31_old * R13_fixed + R32_old * R23_fixed + R33_old * R33_fixed;
        R13 = tmp1;
        R23 = tmp2;
        R33 = tmp3;

        double &px_old = input_data[(ind - 1) * size + 2];
        double &py_old = input_data[(ind - 1) * size + 3];
        double &pz_old = input_data[(ind - 1) * size + 4];

        const double &linear = input_data[ind * size + 0];

        input_data[ind * size + 2] = px_old + R13*linear;
        input_data[ind * size + 3] = py_old + R23*linear;
        input_data[ind * size + 4] = pz_old + R33*linear;

      }
      ind += 1;
    }(), ...);
  }

  template<>
  void forward_kinematics(std::array<JointData, 6> &input_data) {
    forward_kinematics_internal<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT>(
        input_data.data()->joint_data.data());
  }

}