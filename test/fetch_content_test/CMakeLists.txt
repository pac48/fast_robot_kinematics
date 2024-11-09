cmake_minimum_required(VERSION 3.22)
project(fast_forward_kinematics)

set(CMAKE_CXX_STANDARD 20)
if (NOT DEFINED CMAKE_BUILD_TYPE OR "${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
endif ()

# do not build fast_forward_kinematics test for this test project
set(BUILD_TESTING OFF)

include(FetchContent)
FetchContent_Declare(
        fast_forward_kinematics
        GIT_REPOSITORY https://github.com/pac48/fast_robot_kinematics.git
        GIT_TAG fix-cmake
)
FetchContent_MakeAvailable(fast_forward_kinematics)


#TODO use ZIP_LISTS to test all robots
set(URDF_FILE ${CMAKE_SOURCE_DIR}/urdf/robot.urdf)
set(ROOT base_link)
set(TIP grasp_link)
generate_fast_forward_kinematics_library(fast_forward_kinematics_library_{robots} URDF_FILE ${URDF_FILE} ROOT_LINK ${ROOT} TIP_LINK ${TIP})
