#include "chrono"
#include "iostream"
#include "matrix_exponential.hpp"


int main(int arc, char **argv) {

  std::tuple<int, int, int> ss;

  constexpr int iterations = 128 * 128 * 128;
  std::array<double, 7 * 17> input_data = {0};
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; i++) {
    if ((i % 1000) == 0) {
      srand((unsigned int) time(0));
      Eigen::Vector<double, 7*17> rand = Eigen::Vector<double, 7*17>::Random();;
      int ind = 0;
      for (const auto &m: rand) {
        input_data[ind] = m;
        ind++;
      }
    }

    forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(input_data);
//    forward_kinematics<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT,OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(
//        input_data);
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

  int i = 0;
  for (auto &val: input_data) {
    std::cout << i << ": final value: " << val << std::endl;
    i++;
  }

  std::cout << "Time taken by function: " << (double) duration.count() << " nanoseconds" << std::endl;
  std::cout << "Average: " << ((double) duration.count()) / (iterations) << " nanoseconds"
            << std::endl;

  return 0;
}
