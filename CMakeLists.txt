cmake_minimum_required(VERSION 3.22)
project(forward_kinematic)

set(CMAKE_CXX_STANDARD 17)

# Generate the module for the python
find_package(Python REQUIRED COMPONENTS Interpreter)

if(Python_Interpreter_FOUND)
    message(STATUS "Python executable: ${Python_EXECUTABLE}")
else()
    message(FATAL_ERROR "Python interpreter not found.")
endif()

add_custom_command(
    OUTPUT forward_kinematics_test.cpp
    COMMAND ${Python_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/robot_gen.py ${CMAKE_SOURCE_DIR}/src/robot.urdf ${CMAKE_SOURCE_DIR}/scripts/robot_config.cpp.template
    ${CMAKE_CURRENT_BINARY_DIR}/forward_kinematics_test.cpp base_link grasp_link
    DEPENDS ${CMAKE_SOURCE_DIR}/src/robot.urdf  ${CMAKE_SOURCE_DIR}/scripts/robot_config.cpp.template
    COMMENT
    "Running `${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/robot_gen.py ${CMAKE_SOURCE_DIR}/src/robot.urdf ${CMAKE_SOURCE_DIR}/scripts/robot_config.cpp.template
    ${CMAKE_CURRENT_BINARY_DIR}/forward_kinematics_test.cpp base_link grasp_link`"
    VERBATIM
)

add_library(matrix_exponential SHARED forward_kinematics_test.cpp)
target_include_directories(matrix_exponential PUBLIC include)
set_target_properties(matrix_exponential PROPERTIES CMAKE_BUILD_TYPE Release)
target_compile_options(matrix_exponential PUBLIC -O3 -march=native)



add_executable(matrix_exponential_test src/matrix_exponential_test.cpp)
target_link_libraries(matrix_exponential_test PRIVATE matrix_exponential)
