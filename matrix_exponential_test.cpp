#include "chrono"
#include "iostream"
#include "matrix_exponential.hpp"

int main(int arc, char **argv) {

  constexpr int iterations = 128 * 128 * 128 * 10;
  std::array<Eigen::Matrix<double, 3, 3>, 7> sum_matrix;
  Eigen::Vector<double, 7> angles;
  angles
      << -.01, .02, -.03, .04, -.05, .06, -.07;//, .08, -.09, .010, -.011, .012, -.013, .014, -.015, .016, -.017, .018, -.019;
  angles *= arc;

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; i++) {
    forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
        angles.array(), sum_matrix);
    angles.array() += 1E-8*sum_matrix[0](0, 0);
//    forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT,OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT,OP::ROT,OP::ROT,OP::ROT,OP::ROT,OP::ROT,OP::ROT, OP::TRANS>(
//        angles.array(), sum_matrix);
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
