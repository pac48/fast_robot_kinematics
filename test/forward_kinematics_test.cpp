#include "chrono"
#include "iostream"

#include "fast_kinematics.hpp"

int main(int arc, char **argv) {
    constexpr int iterations = 128 * 128;
    std::array<Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS>, iterations> rand_values;
    for (auto &rand_val: rand_values) {
        rand_val = Eigen::Vector<float, FAST_FK_NUMBER_OF_JOINTS>::Random();
    }

    fk_interface::JointDataInterface<fast_fk::JointData> fk_interface;


    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; i++) {
        auto &rand_val = rand_values[i];
        for (int k = 0; k < 128 * 128; k++) {
            fk_interface.set_joints(rand_val);
            fk_interface.forward_kinematics();
        }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

    std::cout << "Time taken by function: " << (float) duration.count() << " nanoseconds" << std::endl;
    std::cout << "Average: " << ((float) duration.count()) / (iterations * 128 * 128) << " nanoseconds"
              << std::endl;

    return 0;
}
