#include "chrono"
#include "iostream"
#include "forward_kinematics.hpp"


double rand_double() {
  double div = RAND_MAX / 2;
  return -1 + (rand() / div);
}

int main(int arc, char **argv) {
  constexpr int iterations = 128 * 128 * 128;
  std::array<double, 6 * 17> input_data = {0};
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; i++) {
    if ((i % 1000) == 0) {
      srand((unsigned int) time(0));
      for (auto & val: input_data) {
        val = rand_double();
      }
    }

    fast_fk::forward_kinematics(input_data);
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
