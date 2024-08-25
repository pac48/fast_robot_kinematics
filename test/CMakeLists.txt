cmake_minimum_required(VERSION 3.22)
project(fast_forward_kinematics)
#enable_language(CUDA)

set(CMAKE_CXX_STANDARD 20)

generate_fast_forward_kinematics_library(${CMAKE_SOURCE_DIR}/test/urdf/ur5.urdf base_link ee_link)
#generate_fast_forward_kinematics_library(${CMAKE_SOURCE_DIR}/test/urdf/robot.urdf base_link grasp_link)
#generate_fast_forward_kinematics_library(${CMAKE_SOURCE_DIR}/test/urdf/pr2.urdf base_link l_gripper_r_finger_tip_link)
if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set_target_properties(fast_forward_kinematics_library PROPERTIES CMAKE_BUILD_TYPE Release)
    target_compile_options(fast_forward_kinematics_library PRIVATE -Ofast -march=native -msse2)
endif ()

add_executable(forward_kinematics_test forward_kinematics_test.cpp)
target_link_libraries(forward_kinematics_test PRIVATE fast_forward_kinematics_library)
add_executable(inverse_kinematics_test inverse_kinematics_test.cpp)
target_link_libraries(inverse_kinematics_test PRIVATE fast_forward_kinematics_library)
