cmake_minimum_required(VERSION 3.22)
project(forward_kinematic)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(tinyxml2)

find_package(ament_cmake REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)

add_executable(forward_kinematic main.cpp)

ament_target_dependencies(forward_kinematic
        rclcpp
        tf2_msgs
        std_msgs
        kdl_parser)

target_link_libraries(forward_kinematic tinyxml2 Eigen3::Eigen)
target_include_directories(forward_kinematic PRIVATE include)

