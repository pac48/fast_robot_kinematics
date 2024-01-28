cmake_minimum_required(VERSION 3.22)
project(fast_forward_kinematics)

set(CMAKE_CXX_STANDARD 17)

#generate_fast_forward_kinematics_library(${CMAKE_SOURCE_DIR}/test/ur5.urdf base_link ee_link)
#generate_fast_forward_kinematics_library(${CMAKE_SOURCE_DIR}/test/robot.urdf base_link grasp_link)
generate_fast_forward_kinematics_library(${CMAKE_SOURCE_DIR}/test/pr2.urdf base_link l_gripper_r_finger_tip_link)
add_executable(matrix_exponential_test matrix_exponential_test.cpp)
target_link_libraries(matrix_exponential_test PRIVATE fast_forward_kinematics_library)
