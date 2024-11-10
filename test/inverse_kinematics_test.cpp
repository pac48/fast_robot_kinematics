#include "chrono"
#include "iostream"

#ifdef USE_FAST_KINEMATICS
#include "fast_inverse_kinematics/fast_kinematics.hpp"
using IK = fast_fk::JointData;
#else

#include "kdl_kinematics.hpp"

using IK = kdl_impl::JointData;
#endif

int main(int arc, char** argv)
{
  unsigned seed = time(0);
  srand(seed);
  rand();

  constexpr int iterations = 128 * 128 * 5 * MULTIPLIER;

  // get target pose
  Eigen::VectorX<float> q_in = Eigen::VectorX<float>::Random(IK::get_num_joints());
  fk_interface::InverseKinematicsInterface<IK> fk_interface;
  fk_interface.set_joints(q_in);
  fk_interface.forward_kinematics();
  Eigen::Matrix<float, 4, 4> tf;
  fk_interface.get_frame(IK::get_num_joints() - 1, tf);

  auto start = std::chrono::high_resolution_clock::now();

  fk_interface::IKSolverStats stats;
  size_t failed = 0;
  size_t succeeded = 0;

  Eigen::VectorX<float> q = 1 * Eigen::VectorX<float>::Random(IK::get_num_joints());
  for(int ind = 0; ind < iterations; ++ind)
  {
    q = Eigen::VectorX<float>::Random(IK::get_num_joints());
    stats = fk_interface.inverse_kinematics(tf, q);
    failed += stats.success == 0;
    succeeded += stats.success == 1;
    //        if (!stats.success) {
    // debug info
    //            std::cout << " failed!!" << std::endl;
    //            std::cout << "q: " << q.transpose() << std::endl;
    //            std::cout << "q_in: " << q_in.transpose() << std::endl;
    //            std::cout << stats.niter << " iterations" << std::endl;
    //            std::cout << "f(x) = " << stats.fx << std::endl;
    //            std::cout << "||grad|| = " << stats.grad_norm << std::endl;
    //            std::cout << "error: " << stats.what << std::endl;
    //            continue;
    //        }
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "Time taken by function: " << (double)duration.count() << " microseconds"
            << std::endl;
  std::cout << "Average: " << ((double)duration.count()) / (iterations) << " microseconds"
            << std::endl;
  std::cout << "Percent success: " << ((double)(100.0 * succeeded) / (failed + succeeded))
            << "%" << std::endl;

  return 0;
}
