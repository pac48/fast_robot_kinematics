# fast_robot_kinematics

Compile a URDF into an optimized C++ library for fast forward kinematics.

Slides from CPPCon 2024 can be found [here](https://pac48.github.io/cpp-con-2024/)

# Usage

```cmake
include(FetchContent)
FetchContent_Declare(
        fast_forward_kinematics
        GIT_REPOSITORY https://github.com/pac48/fast_robot_kinematics.git
        GIT_TAG main
)
FetchContent_MakeAvailable(fast_forward_kinematics)

set(URDF_FILE ${CMAKE_SOURCE_DIR}/urdf/robot.urdf)
set(ROOT base_link)
set(TIP grasp_link)
generate_fast_forward_kinematics_library(fast_forward_kinematics_library
                                         URDF_FILE ${URDF_FILE}
                                         ROOT_LINK ${ROOT}
                                         TIP_LINK ${TIP})
```
