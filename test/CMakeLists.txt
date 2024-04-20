cmake_minimum_required(VERSION 3.22)
project(fast_forward_kinematics)
enable_language(CUDA)

set(CMAKE_CXX_STANDARD 17)

#generate_fast_forward_kinematics_library(${CMAKE_SOURCE_DIR}/test/ur5.urdf base_link ee_link)
generate_fast_forward_kinematics_library(${CMAKE_SOURCE_DIR}/test/robot.urdf base_link grasp_link)
#generate_fast_forward_kinematics_library(${CMAKE_SOURCE_DIR}/test/pr2.urdf base_link l_gripper_r_finger_tip_link)
target_compile_options(fast_forward_kinematics_library PRIVATE -Ofast -march=native)

add_executable(forward_kinematics_test forward_kinematics_test.cpp)
target_link_libraries(forward_kinematics_test PRIVATE fast_forward_kinematics_library)
add_library(cuda_forward_kinematics libforward_kinematics.cu)
target_include_directories(cuda_forward_kinematics PRIVATE ../include)
