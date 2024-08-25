#include "chrono"
#include "iostream"

#include "forward_kinematics_eigen.hpp"
#include "inverse_kinematics_eigen.hpp"


int main(int arc, char **argv) {
    unsigned seed = time(0);
    srand(seed);
    rand();

    constexpr int iterations = 128 * 128 * 10;

    // get target pose
    Eigen::VectorX<float> q_in = Eigen::VectorX<float>::Random(FAST_FK_NUMBER_OF_JOINTS);
    fast_fk::JointData joints;
    joints.set_joints(q_in);
    fast_fk::forward_kinematics(joints);
    Eigen::Matrix<float, 4, 4> tf;
    joints.get_frame(FAST_FK_NUMBER_OF_JOINTS - 1, tf);
    Eigen::Matrix<float, 3, 3> target_rot = tf.block<3, 3>(0, 0);
    Eigen::Vector<float, 3> target_pose = tf.block<3, 1>(0, 3);

    LBFGSpp::LBFGSParam<float> param;
    param.epsilon = 1E-3;
    param.epsilon_rel = 1E-3;
    LBFGSpp::LBFGSSolver<float> solver(param);
    fast_fk::InverseKinematics fun(target_rot, target_pose);

    auto start = std::chrono::high_resolution_clock::now();

    float fx;
    int niter;
    Eigen::VectorX<float> q = 1 * Eigen::VectorX<float>::Random(FAST_FK_NUMBER_OF_JOINTS);
    for (int ind = 0; ind < iterations; ++ind) {
        q = Eigen::VectorX<float>::Random(FAST_FK_NUMBER_OF_JOINTS);
        try {
            niter = solver.minimize(fun, q, fx);
        } catch (const std::runtime_error &e) {
            std::cout << " failed!!" << std::endl;
            std::cout << "q: " << q.transpose() << std::endl;
            std::cout << "q_in: " << q_in.transpose() << std::endl;
            std::cout << niter << " iterations" << std::endl;
            std::cout << "f(x) = " << fx << std::endl;
            std::cout << "||grad|| = " << solver.final_grad_norm() << std::endl;
            std::cout << "error: " << e.what() << std::endl;
            continue;
        }
//        std::cout << "f(x) = " << fx << std::endl;
//        std::cout << niter << " iterations" << std::endl;
//        std::cout << "f(x) = " << fx << std::endl;
//        std::cout << "Done"<< std::endl;
//        return 0;

//        std::cout << niter << " iterations" << std::endl;
//        std::cout << "f(x) = " << fx << std::endl;
//        std::cout << "||grad|| = " << solver.final_grad_norm() << std::endl;
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "Time taken by function: " << (double ) duration.count() << " microseconds" << std::endl;
    std::cout << "Average: " << ((double ) duration.count()) / (iterations) << " microseconds"
              << std::endl;

    return 0;
}
