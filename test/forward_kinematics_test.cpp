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
    constexpr int iterations = 128 * 128;
    constexpr int multiplier = 128*64;
    std::array<Eigen::Vector<float, KI::get_num_joints()>, iterations> rand_values;
    for (auto &rand_val: rand_values) {
        rand_val = Eigen::Vector<float, KI::get_num_joints()>::Random();
    }

    fk_interface::JointDataInterface<KI> fk_interface;


    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; i++) {
        auto &rand_val = rand_values[i];
        for (int k = 0; k < multiplier; k++) {
            fk_interface.set_joints(rand_val);
            fk_interface.forward_kinematics();
        }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

    std::cout << "Time taken by function: " << (float) duration.count() << " nanoseconds" << std::endl;
    std::cout << "Average: " << ((float) duration.count()) / (iterations * multiplier) << " nanoseconds"
              << std::endl;

    return 0;
}
