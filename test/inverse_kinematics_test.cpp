#include "chrono"
#include "iostream"

#ifdef FAST_FK_USE_EIGEN

#include "forward_kinematics_eigen.hpp"
#include "inverse_kinematics_eigen.hpp"

#else
#include "forward_kinematics.hpp"
#endif

int main(int arc, char **argv) {
    unsigned seed = time(0);
    srand(seed);
    rand();

    constexpr int iterations = 128 * 128 * 10;

    // get target pose
    Eigen::VectorXd q_in = Eigen::VectorXd::Random(FAST_FK_NUMBER_OF_JOINTS);
    fast_fk::JointData joints;
    joints.set_joints(q_in);
    fast_fk::forward_kinematics(joints);
    Eigen::Matrix<double, 4, 4> tf;
    joints.get_frame(FAST_FK_NUMBER_OF_JOINTS - 1, tf);
    Eigen::Matrix<double, 3, 3> target_rot = tf.block<3, 3>(0, 0);
    Eigen::Vector<double, 3> target_pose = tf.block<3, 1>(0, 3);

    LBFGSpp::LBFGSParam<double> param;
    LBFGSpp::LBFGSSolver<double> solver(param);
    fast_fk::InverseKinematics fun(target_rot, target_pose);

    auto start = std::chrono::high_resolution_clock::now();

    double fx;
    int niter;
    Eigen::VectorXd q = 1 * Eigen::VectorXd::Random(FAST_FK_NUMBER_OF_JOINTS);
    for (int ind = 0; ind < iterations; ++ind) {
        q = Eigen::VectorXd::Random(FAST_FK_NUMBER_OF_JOINTS);
        try {
            niter = solver.minimize(fun, q, fx);
        } catch (const std::runtime_error &e) {
            std::cout << " failed!!" << std::endl;
            std::cout << "q: " << q.transpose() << std::endl;
            std::cout << "q_in: " << q_in.transpose() << std::endl;
            std::cout << niter << " iterations" << std::endl;
            std::cout << "f(x) = " << fx << std::endl;
            std::cout << "||grad|| = " << solver.final_grad_norm() << std::endl;
            continue;
        }

//        std::cout << niter << " iterations" << std::endl;
//        std::cout << "f(x) = " << fx << std::endl;
//        std::cout << "||grad|| = " << solver.final_grad_norm() << std::endl;
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "Time taken by function: " << (double) duration.count() << " microseconds" << std::endl;
    std::cout << "Average: " << ((double) duration.count()) / (iterations) << " microseconds"
              << std::endl;

    return 0;
}
