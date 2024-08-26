#include "chrono"
#include "iostream"

#ifdef USE_FAST_KINEMATICS
#include "fast_kinematics.hpp"
using KI = fast_fk::JointData;
#else
#include "kdl_kinematics.hpp"
using KI = kdl_impl::JointData;
#endif



int main(int arc, char **argv) {
    unsigned seed = time(0);
    srand(seed);
    rand();

    constexpr int iterations = 128 * 128 * 10;

    // get target pose
    Eigen::VectorX<float> q_in = Eigen::VectorX<float>::Random(KI::get_num_joints());
    fk_interface::JointDataInterface<KI> fk_interface;
    fk_interface.set_joints(q_in);
    fk_interface.forward_kinematics();
    Eigen::Matrix<float, 4, 4> tf;
    fk_interface.get_frame(KI::get_num_joints() - 1, tf);
//    Eigen::Matrix<float, 3, 3> target_rot = tf.block<3, 3>(0, 0);
//    Eigen::Vector<float, 3> target_pose = tf.block<3, 1>(0, 3);

    auto start = std::chrono::high_resolution_clock::now();

    fk_interface::IKSolverStats stats;

    Eigen::VectorX<float> q = 1 * Eigen::VectorX<float>::Random(KI::get_num_joints());
    for (int ind = 0; ind < iterations; ++ind) {
        q = Eigen::VectorX<float>::Random(KI::get_num_joints());
        stats = fk_interface.inverse_kinematics(tf, q);
        if (!stats.success) {
            std::cout << " failed!!" << std::endl;
            std::cout << "q: " << q.transpose() << std::endl;
            std::cout << "q_in: " << q_in.transpose() << std::endl;
            std::cout << stats.niter << " iterations" << std::endl;
            std::cout << "f(x) = " << stats.fx << std::endl;
            std::cout << "||grad|| = " << stats.grad_norm << std::endl;
            std::cout << "error: " << stats.what << std::endl;
            continue;
        }

    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "Time taken by function: " << (double) duration.count() << " microseconds" << std::endl;
    std::cout << "Average: " << ((double) duration.count()) / (iterations) << " microseconds"
              << std::endl;

    return 0;
}
