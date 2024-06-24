#include "chrono"
#include "iostream"

#ifdef FAST_FK_USE_EIGEN

#include "forward_kinematics_eigen.hpp"
#include "inverse_kinematics_eigen.hpp"

#else
#include "forward_kinematics.hpp"
#endif

int main(int arc, char **argv) {
    constexpr int iterations = 128;

    // get target pose
    Eigen::VectorXd q = Eigen::VectorXd::Random(FAST_FK_NUMBER_OF_JOINTS);
    fast_fk::JointData joints;
    joints.set_joints(q);
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
    for (int ind = 0; ind < 100; ++ind) {
        q = Eigen::VectorXd::Random(FAST_FK_NUMBER_OF_JOINTS);
        niter = solver.minimize(fun, q, fx);
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

    std::cout << "Time taken by function: " << (double) duration.count() << " nanoseconds" << std::endl;
    std::cout << "Average: " << ((double) duration.count()) / (iterations * 128 * 128) << " nanoseconds"
              << std::endl;

    return 0;
}
