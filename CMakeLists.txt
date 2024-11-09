cmake_minimum_required(VERSION 3.22)
project(fast_forward_kinematics)

set(FFK_VERSION_MAJOR 0)
set(FFK_VERSION_MINOR 1)
set(FFK_VERSION ${FFK_VERSION_MAJOR}.${FFK_VERSION_MINOR})

option(BUILD_TESTING "Build fast_forward_kinematics tests" OFF)

# Include GNUInstallDirs to get canonical paths
include(GNUInstallDirs)
include(CTest)

# Add cmake macro to project
include(cmake/fast_forward_kinematics.cmake)

if (BUILD_TESTING)
    add_subdirectory(test)
endif ()


include(CMakePackageConfigHelpers)

configure_package_config_file(fast_forward_kinematics-config.cmake.in fast_forward_kinematics-config.cmake
        INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/fast_forward_kinematics"
        PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
        NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(fast_forward_kinematics-config-version.cmake
        VERSION ${FFK_VERSION} COMPATIBILITY AnyNewerVersion)

install(FILES
        "${CMAKE_BINARY_DIR}/fast_forward_kinematics-config.cmake"
        "${CMAKE_BINARY_DIR}/fast_forward_kinematics-config-version.cmake"
        "${CMAKE_SOURCE_DIR}/cmake/fast_forward_kinematics.cmake"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/fast_forward_kinematics/cmake")