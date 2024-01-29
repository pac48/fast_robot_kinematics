#include "chrono"
#include "iostream"

#ifdef FAST_FK_USE_EIGEN
#include "forward_kinematics_eigen.hpp"
#else
#include "forward_kinematics.hpp"
#endif

double rand_double() {
  double div = RAND_MAX / 2;
  return -1 + (rand() / div);
}

int main(int arc, char **argv) {
  constexpr int iterations = 128 * 128 * 128;
  constexpr size_t num_joints = FAST_FK_NUMBER_OF_JOINTS;
  std::array<fast_fk::JointData, num_joints> joints = {0};
  auto start = std::chrono::high_resolution_clock::now();
  double rand_val = 0;
  for (int i = 0; i < iterations; i++) {
    if ((i % 1000) == 0) {
      srand((unsigned int) time(0));
      rand_val = rand_double();
    }
    rand_val *= 0.999;
    for (auto k = 0; k < num_joints; ++k) {
//      joints[k].set_joint(rand_val, sqrt(1.0 - rand_val * rand_val));
//      rand_val = 3.141592/2;
      joints[k].set_joint(rand_val);
    }

    fast_fk::forward_kinematics(joints);
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

//  for (auto i = 0; i < num_joints; ++i) {
//    std::cout << i << ": final value: " << joints[i].joint_data[4] << std::endl;
//    i++;
//  }

  std::cout << "Time taken by function: " << (double) duration.count() << " nanoseconds" << std::endl;
  std::cout << "Average: " << ((double) duration.count()) / (iterations) << " nanoseconds"
            << std::endl;

  return 0;
}
