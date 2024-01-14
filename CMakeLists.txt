cmake_minimum_required(VERSION 3.22)
project(forward_kinematic)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(tinyxml2)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
add_executable(forward_kinematic main.cpp)
target_link_libraries(forward_kinematic tinyxml2 Eigen3::Eigen)
target_include_directories(forward_kinematic PRIVATE include)

add_executable(speed_test speed_test.cpp)
set_target_properties(speed_test PROPERTIES CMAKE_BUILD_TYPE Release)
target_compile_options(speed_test PRIVATE -O3 -ffast-math -march=native -masm=intel)
target_link_libraries(speed_test Eigen3::Eigen)



add_library(matrix_exponential STATIC matrix_exponential.cpp)
target_link_libraries(matrix_exponential PUBLIC Eigen3::Eigen)
set_target_properties(matrix_exponential PROPERTIES CMAKE_BUILD_TYPE Release)
target_compile_options(matrix_exponential PUBLIC -O3 -ffast-math -march=native -msse3)


add_executable(matrix_exponential_test matrix_exponential_test.cpp)
target_link_libraries(matrix_exponential_test PRIVATE matrix_exponential)
