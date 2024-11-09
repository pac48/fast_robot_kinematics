#pragma once

#include "memory"

#include <Eigen/Core>
#include <Eigen/Dense>

#ifdef FAST_FK_USE_IK

#include "LBFGS.h"

#include "kinematics_interface.hpp"

namespace fast_fk::internal {
    class InverseKinematics {
    public:
        Eigen::Matrix<float, 3, 3> target_rot_;
        Eigen::Vector<float, 3> target_pose_;
        std::array<std::array<float, 16>, FAST_FK_NUMBER_OF_JOINTS> joint_data = {0};
        std::unique_ptr<LBFGSpp::LBFGSSolver<float>> solver;
        LBFGSpp::LBFGSParam<float> param;

        InverseKinematics(const Eigen::Matrix<float, 3, 3> &target_rot,
                          const Eigen::Vector<float, 3> &target_pose) : target_rot_{target_rot},
                                                                        target_pose_{target_pose} {
            param.epsilon = 1E-3;
            param.epsilon_rel = 1E-3;
            param.max_iterations = 30;

            solver = std::make_unique<LBFGSpp::LBFGSSolver<float >>(param);
        }

        fk_interface::IKSolverStats
        inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess) {
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

        float operator()(const Eigen::VectorX<float> &q, Eigen::VectorX<float> &grad);


    };
}
#else
namespace fast_fk::internal {
    class InverseKinematics {
    public:
        InverseKinematics(const Eigen::Matrix<float, 3, 3> &target_rot,
                                             const Eigen::Vector<float, 3> &target_pose){}

        fk_interface::IKSolverStats
        inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess) {
            throw std::logic_error("Function not implemented.");
            return {};
        }

        float operator()(const Eigen::VectorX<float> &q, Eigen::VectorX<float> &grad);

    };
}
#endif
