cmake_minimum_required(VERSION 3.22)
project(fast_forward_kinematics)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)

option(BUILD_TESTING "Build fast_forward_kinematics tests" OFF)

# Include GNUInstallDirs to get canonical paths
include(GNUInstallDirs)
include(CTest)

# Generate the module for the python
include(cmake/fast_forward_kinematics-extras.cmake)

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/fast_forward_kinematics-extras.cmake
        "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/fast_forward_kinematics-extras.cmake" @ONLY
)

install(DIRECTORY include DESTINATION include/)

if (BUILD_TESTING)
    add_subdirectory(test)
endif ()