#include "kdl_kinematics.hpp"

namespace kdl_impl {

    void JointData::forward_kinematics() {
        // create forward kinematics solver
        fk_pos_solver->JntToCart(q, frame, ee_ind);
        transformKDLToEigen(frame, transform);
        int o = 0;
    }
}