cmake_minimum_required(VERSION 3.22)
project(fast_forward_kinematics)

# Generate the module for the python
include(cmake/fast_forward_kinematics-extras.cmake)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/fast_forward_kinematics-extras.cmake
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/fast_forward_kinematics-extras.cmake" @ONLY
)

install(DIRECTORY include DESTINATION include/)

add_subdirectory(test)
