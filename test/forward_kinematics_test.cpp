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
    constexpr int multiplier = 8*8*32;
    auto rand_values = std::make_unique<std::array<Eigen::Vector<float, KI::get_num_joints()>, fast_fk::batch_size>>();
    for (auto &rand_val: *rand_values) {
        rand_val = Eigen::Vector<float, KI::get_num_joints()>::Random();
    }

    auto fk_interface = std::make_unique<fk_interface::ForwardKinematicsInterface<KI>>();
    Eigen::Matrix<float, 4, 4> tf;

    for (int i = 0; i < fast_fk::batch_size; i++) {
        fk_interface->set_joints(i, rand_values->operator[](i));
    }
    auto start = std::chrono::high_resolution_clock::now();

    for (int k = 0; k < multiplier; k++) {
        fk_interface->forward_kinematics();

    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

    std::cout << "Time taken by function: " << (float) duration.count() << " nanoseconds" << std::endl;
    std::cout << "Average: " << ((float) duration.count()) / (fast_fk::batch_size * multiplier) << " nanoseconds"
              << std::endl;

    return 0;
}
