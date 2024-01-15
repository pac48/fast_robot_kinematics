#include "chrono"
#include "iostream"
#include "matrix_exponential.hpp"


int main(int arc, char **argv) {

  constexpr int iterations = 128 * 128 * 128;
  Eigen::Matrix<double, 7, 7> angles = Eigen::Matrix<double, 7, 7>::Ones();

  std::array<std::array<Eigen::Vector<double, 4>, 4>, 7> sum_matrix;
  for (auto &m: sum_matrix) {
    for (auto &c: m) {
      c = Eigen::Vector<double, 4>::Zero();
    }
  }
  angles.col(0)
      << -.01, .02, -.03, .04, -.05, .06, -.07, .08, -.09, .010, -.011, .012, -.013, .014, -.015, .016, -.017, .018, -.019;
  angles *= arc;

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; i++) {
//    for (auto v = 0 ; v < 7;v++){
//      forward_kinematics<OP::ROT>(angles.array(), sum_matrix);
//    }

    forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(angles.array(), sum_matrix);

//    forward_kinematics_internal<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
//        angles, sum_matrix);
//    angles.array() += 1E-4 * sum_matrix[0](0, 0);
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

  for (auto &val: angles.col(0)) {
    std::cout << "final value: " << val << std::endl;
  }

  std::cout << "Time taken by function: " << (double) duration.count() << " nanoseconds" << std::endl;
  std::cout << "Average: " << ((double) duration.count()) / (iterations) << " nanoseconds"
            << std::endl;

  return 0;
}
