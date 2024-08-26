#pragma once

#include "memory"
#include "fstream"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "chainfksolverpos_recursive.hpp"
#include "chainiksolverpos_lma.hpp"
#include "chainfksolvervel_recursive.hpp"
#include "chainjnttojacsolver.hpp"
#include "treejnttojacsolver.hpp"
#include "kdl_parser/kdl_parser.hpp"

#include "kinematics_interface.hpp"


#define STRINGIZE(x) #x
#define STRINGIZE_VALUE_OF(x) STRINGIZE(x)

namespace kdl_impl {


    struct JointData {
        static constexpr size_t get_num_joints() {
            return NUMBER_OF_JOINTS;
        }

        JointData() {
            std::string urdf_file = STRINGIZE_VALUE_OF(URDF_FILE);
            std::string robot_description;
            {
                std::fstream f;
                f.open(urdf_file);
                std::stringstream ss;
                ss << f.rdbuf();
                robot_description = ss.str();
            }
            // create kinematic chain
            kdl_parser::treeFromString(robot_description, robot_tree);
            std::string root_name = STRINGIZE_VALUE_OF(ROOT);
            std::string tip_name = STRINGIZE_VALUE_OF(TIP);

            if (!robot_tree.getChain(root_name, tip_name, chain)) {
                throw std::runtime_error("failed to load robot from URDF");
            }
            auto num_joints_ = chain.getNrOfJoints();
            assert(num_joints_ == NUMBER_OF_JOINTS);
            q = KDL::JntArray(num_joints_);
            q_out = q;

            fk_pos_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
            ik_solver_lma = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain);

            for (auto i = 0; i < chain.getNrOfSegments(); ++i) {
                if (chain.getSegment(i).getName() == tip_name) {
                    ee_ind = i + 1;
                    break;
                }
            }
            if (ee_ind == -1) {
                throw std::runtime_error("The tip `" + tip_name + " is missing from the chain.");
            }


        }


        void set_joint(size_t ind, float value) {
//            joint_data[ind][0] = std::sin(value);
//            joint_data[ind][1] = std::cos(value);
        }

        void set_joints(const Eigen::Vector<float, NUMBER_OF_JOINTS> &values) {
#pragma unroll
            for (auto ind = 0; ind < NUMBER_OF_JOINTS; ++ind) {
//                joint_data[ind][0] = std::sin(values[ind]);
//                joint_data[ind][1] = std::cos(values[ind]);
            }
        }

        void set_joint(size_t ind, float sin_t, float cos_t) {
//            joint_data[ind][0] = sin_t;
//            joint_data[ind][1] = cos_t;
        }

        void set_joints(const float *sin_values, const float *cos_values) {
#pragma unroll
            for (auto ind = 0; ind < NUMBER_OF_JOINTS; ++ind) {
//                joint_data[ind][0] = sin_values[ind];
//                joint_data[ind][1] = cos_values[ind];
            }
        }

        void set_joints(const Eigen::Vector<float, NUMBER_OF_JOINTS> &sin_values,
                        const Eigen::Vector<float, NUMBER_OF_JOINTS> &cos_values) {
#pragma unroll
            for (auto ind = 0; ind < NUMBER_OF_JOINTS; ++ind) {
//                joint_data[ind][0] = sin_values[ind];
//                joint_data[ind][1] = cos_values[ind];
            }
        }


        [[nodiscard]] float get_joint(size_t ind) const {
            return 0;
//            return atan2f(joint_data[ind][0], joint_data[ind][1]);
        }

        void get_joints(Eigen::Vector<float, NUMBER_OF_JOINTS> &values) const {
#pragma unroll
            for (auto ind = 0; ind < NUMBER_OF_JOINTS; ++ind) {
//                values[ind] = atan2f(joint_data[ind][0], joint_data[ind][1]);
            }
        }


        void get_frame(size_t index, Eigen::Matrix<float, 4, 4> &tf) const;

        void forward_kinematics();

        fk_interface::IKSolverStats
        inverse_kinematics(Eigen::Matrix<float, 4, 4> &transform, Eigen::VectorX<float> &q_guess);


        Eigen::Matrix<float, 4, 4> transform;
        KDL::JntArray q;
        KDL::JntArray q_out;
        int ee_ind = -1;

        KDL::Tree robot_tree;
        KDL::Chain chain;
        KDL::Frame frame;
        std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
        std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_lma;

    };

}
