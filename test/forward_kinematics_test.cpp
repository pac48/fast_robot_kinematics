#include "chrono"
#include "iostream"

#ifdef FAST_FK_USE_EIGEN

#include "forward_kinematics_eigen.hpp"

#else
#include "forward_kinematics.hpp"
#endif

int main(int arc, char **argv) {
    constexpr int iterations = 128 * 128;
    std::array<Eigen::Vector<double, FAST_FK_NUMBER_OF_JOINTS>, iterations> rand_values;
    for (auto &rand_val: rand_values) {
        rand_val = Eigen::Vector<double, FAST_FK_NUMBER_OF_JOINTS>::Random();
    }

    fast_fk::JointData joints;
    auto start = std::chrono::high_resolution_clock::now();
    for (int k = 0; k < 128*128 ; k++) {
        for (int i = 0; i < iterations; i++) {
            auto &rand_val = rand_values[i];
            joints.set_joints(rand_val, (1.0 - rand_val.array() * rand_val.array()).sqrt());
            fast_fk::forward_kinematics(joints);

            //    Eigen::Matrix4d transform;
//    for (auto ind_debug = 0 ; ind_debug<6; ind_debug++){
//      joints.get_frame(ind_debug, transform);
//      std::cout << transform <<"\n\n";
//    }

//    break;
        }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

    std::cout << "Time taken by function: " << (double) duration.count() << " nanoseconds" << std::endl;
    std::cout << "Average: " << ((double) duration.count()) / (iterations*128*128) << " nanoseconds"
              << std::endl;

    return 0;
}
