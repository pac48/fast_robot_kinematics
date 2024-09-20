#include "fast_inverse_kinematics.hpp"

#ifdef FAST_FK_USE_IK

namespace fast_fk::internal {
    // input_data: sin(t) cos(t)  px py pz R11, R12, R13...
    void forward_kinematics_internal(float *input_data, size_t size);

    InverseKinematics::InverseKinematics(const Eigen::Matrix<float, 3, 3> &target_rot,
                                         const Eigen::Vector<float, 3> &target_pose) : target_rot_{target_rot},
                                                                                       target_pose_{target_pose} {
        param.epsilon = 1E-3;
        param.epsilon_rel = 1E-3;
        param.max_iterations = 30;

        solver = std::make_unique<LBFGSpp::LBFGSSolver<float >>(param);
    }

    fk_interface::IKSolverStats
    InverseKinematics::inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess) {
        target_rot_ = transform.block<3, 3>(0, 0);
        target_pose_(0) = transform(0, 3);
        target_pose_(1) = transform(1, 3);
        target_pose_(2) = transform(2, 3);

        float fx = 1E10;
        int niter;

        try {
            niter = solver->minimize(*this, q_guess, fx);
        } catch (const std::runtime_error &e) {
            return {fx, niter, solver->final_grad_norm(), false, e.what()};
        }

        return {fx, niter, solver->final_grad_norm(), true, ""};
    }
}

#else

namespace fast_fk::internal {
    InverseKinematics::InverseKinematics(const Eigen::Matrix<float, 3, 3> &target_rot,
                                         const Eigen::Vector<float, 3> &target_pose){}

    fk_interface::IKSolverStats
    InverseKinematics::inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess) {
        throw std::logic_error("Function not implemented.");
        return {};
    }

};
#endif