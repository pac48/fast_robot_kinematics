include(CMakeParseArguments)
function(generate_fast_forward_kinematics_library_common target_name)
    cmake_parse_arguments(ARG "" "URDF_FILE;ROOT_LINK;TIP_LINK;EXT" "" ${ARGN})
    if (ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "generate_fast_forward_kinematics_library_common() called with invalid arguments.")
    endif ()

    # Set cmake build type to Release if not specified
    if (NOT DEFINED CMAKE_BUILD_TYPE OR "${CMAKE_BUILD_TYPE}" STREQUAL "")
        set(CMAKE_BUILD_TYPE Release)
    endif ()

    find_package(Python REQUIRED COMPONENTS Interpreter)
    if (Python_Interpreter_FOUND)
        message(STATUS "Python executable: ${Python_EXECUTABLE}")
    else ()
        message(FATAL_ERROR "Python interpreter not found.")
    endif ()

    execute_process(
            COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/code_generation/get_num_joints.py ${ARG_URDF_FILE} ${ARG_ROOT_LINK} ${ARG_TIP_LINK}
            OUTPUT_VARIABLE FAST_FK_NUMBER_OF_JOINTS
            OUTPUT_STRIP_TRAILING_WHITESPACE
            COMMAND_ECHO STDOUT
    )

    add_custom_command(
            OUTPUT forward_kinematics_lib.${ARG_EXT}
            COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/code_generation/robot_gen.py ${ARG_URDF_FILE} ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/code_generation/robot_config.${ARG_EXT}.template
            ${CMAKE_CURRENT_BINARY_DIR}/forward_kinematics_lib.${ARG_EXT} ${ARG_ROOT_LINK} ${ARG_TIP_LINK}
            DEPENDS ${ARG_URDF_FILE} ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/code_generation/robot_config.${ARG_EXT}.template
            COMMENT
            "Running `${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/code_generation/robot_gen.py
                      ${ARG_URDF_FILE}
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/code_generation/robot_config.cpp.template
                      ${CMAKE_CURRENT_BINARY_DIR}/forward_kinematics_test.cpp
                      ${ROOT_LINK}
            ${TIP_LINK}`"
            VERBATIM
    )
    add_custom_target(code_generation DEPENDS forward_kinematics_lib.${ARG_EXT})
    add_library(${target_name} SHARED forward_kinematics_lib.${ARG_EXT})
    add_dependencies(${target_name} code_generation)

    target_include_directories(${target_name} PUBLIC ${CMAKE_SOURCE_DIR}/include)
    target_compile_definitions(${target_name} PUBLIC "${FAST_FK_NUMBER_OF_JOINTS}")


endfunction()


macro(ffk_failed_arg_parse)
    message(FATAL_ERROR "generate_fast_forward_kinematics_library() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}. "
            "Expected arguments: generate_fast_forward_kinematics_library(target_name URDF_FILE val1 ROOT_LINK val2 TIP_LINK val3)")
endmacro()

function(generate_fast_forward_kinematics_library target_name)
    cmake_parse_arguments(ARG "" "URDF_FILE;ROOT_LINK;TIP_LINK" "" ${ARGN})
    if (ARG_UNPARSED_ARGUMENTS)
        ffk_failed_arg_parse()
    endif ()

    include(ExternalProject)
    ExternalProject_Add(
            LBFGSpp
            PREFIX ${CMAKE_BINARY_DIR}/LBFGSpp
            GIT_REPOSITORY https://github.com/yixuan/LBFGSpp.git
            GIT_TAG v0.3.0
            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR} -DBUILD_TESTING=OFF
    )
    ExternalProject_Get_Property(LBFGSpp source_dir)
    set(LBFGSppIncludeDir ${source_dir}/include)

    generate_fast_forward_kinematics_library_common(fast_forward_kinematics_library
            URDF_FILE "${ARG_URDF_FILE}"
            ROOT_LINK "${ARG_ROOT_LINK}"
            TIP_LINK "${ARG_TIP_LINK}"
            EXT "cpp")

    add_dependencies(fast_forward_kinematics_library LBFGSpp)
    target_compile_definitions(fast_forward_kinematics_library PUBLIC FAST_FK_USE_IK)
    target_include_directories(fast_forward_kinematics_library PUBLIC ${LBFGSppIncludeDir})
#    find_package(Eigen3 3.3 NO_MODULE)
    if(NOT Eigen3_FOUND)
        include(FetchContent)
        set(PROJECT_BUILD_TESTING ${BUILD_TESTING})
        set(BUILD_TESTING OFF)
        FetchContent_Declare(
                eigen
                GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
                GIT_TAG        3.4.0
        )
        FetchContent_MakeAvailable(eigen)
        set(BUILD_TESTING ${PROJECT_BUILD_TESTING})

    endif ()
    target_link_libraries(fast_forward_kinematics_library PUBLIC Eigen3::Eigen)

    target_compile_options(fast_forward_kinematics_library PUBLIC -Ofast -march=native)

endfunction()

function(generate_fast_forward_kinematics_library_cuda target_name)
    cmake_parse_arguments(ARG "" "URDF_FILE;ROOT_LINK;TIP_LINK" "" ${ARGN})
    if (ARG_UNPARSED_ARGUMENTS)
        ffk_failed_arg_parse()
    endif ()
    enable_language(CUDA)
    generate_fast_forward_kinematics_library_common(fast_forward_kinematics_library
            URDF_FILE "${ARG_URDF_FILE}"
            ROOT_LINK "${ARG_ROOT_LINK}"
            TIP_LINK "${ARG_TIP_LINK}"
            EXT "cu")
endfunction()