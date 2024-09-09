function(generate_fast_forward_kinematics_library_common URDF_FILE ROOT_LINK TIP_LINK EXT)

    find_package(Python REQUIRED COMPONENTS Interpreter)
    if (Python_Interpreter_FOUND)
        message(STATUS "Python executable: ${Python_EXECUTABLE}")
    else ()
        message(FATAL_ERROR "Python interpreter not found.")
    endif ()

    execute_process(
            COMMAND ${Python_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/get_num_joints.py ${URDF_FILE} ${ROOT_LINK} ${TIP_LINK}
            OUTPUT_VARIABLE FAST_FK_NUMBER_OF_JOINTS
            OUTPUT_STRIP_TRAILING_WHITESPACE
            COMMAND_ECHO STDOUT
    )

    add_custom_command(
            OUTPUT forward_kinematics_lib.${EXT}
            COMMAND ${Python_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/robot_gen.py ${URDF_FILE} ${CMAKE_SOURCE_DIR}/scripts/robot_config.${EXT}.template
            ${CMAKE_CURRENT_BINARY_DIR}/forward_kinematics_lib.${EXT} ${ROOT_LINK} ${TIP_LINK}
            DEPENDS ${URDF_FILE} ${CMAKE_SOURCE_DIR}/scripts/robot_config.${EXT}.template
            COMMENT
            "Running `${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/robot_gen.py
                      ${URDF_FILE}
                      ${CMAKE_SOURCE_DIR}/scripts/robot_config.cpp.template
                      ${CMAKE_CURRENT_BINARY_DIR}/forward_kinematics_test.cpp
                      ${ROOT_LINK}
                      ${TIP_LINK}`"
            VERBATIM
    )
    add_custom_target(code_generation DEPENDS forward_kinematics_lib.${EXT})

    set(sources ${CMAKE_SOURCE_DIR}/src/fast_kinematics.cpp ${CMAKE_SOURCE_DIR}/src/fast_inverse_kinematics.cpp)

    add_library(fast_forward_kinematics_library SHARED forward_kinematics_lib.${EXT} ${sources})
    add_dependencies(fast_forward_kinematics_library code_generation)

    target_include_directories(fast_forward_kinematics_library PUBLIC ${CMAKE_SOURCE_DIR}/include)
    target_compile_definitions(fast_forward_kinematics_library PUBLIC "${FAST_FK_NUMBER_OF_JOINTS}")
    find_package(Eigen3 3.3 NO_MODULE)
    target_link_libraries(fast_forward_kinematics_library PUBLIC Eigen3::Eigen)

endfunction()

function(generate_fast_forward_kinematics_library URDF_FILE ROOT_LINK TIP_LINK)
    include(ExternalProject)
    ExternalProject_Add(
            LBFGSpp
            PREFIX ${CMAKE_BINARY_DIR}/LBFGSpp
            GIT_REPOSITORY https://github.com/yixuan/LBFGSpp.git
            GIT_TAG v0.3.0
            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
    )
    ExternalProject_Get_Property(LBFGSpp source_dir)
    set(LBFGSppIncludeDir ${source_dir}/include)

    generate_fast_forward_kinematics_library_common(${URDF_FILE} ${ROOT_LINK} ${TIP_LINK} "cpp")

    add_dependencies(fast_forward_kinematics_library LBFGSpp)
    target_compile_definitions(fast_forward_kinematics_library PUBLIC FAST_FK_USE_IK)
    target_include_directories(fast_forward_kinematics_library PUBLIC ${LBFGSppIncludeDir})

endfunction()

function(generate_fast_forward_kinematics_library_cuda URDF_FILE ROOT_LINK TIP_LINK)
    generate_fast_forward_kinematics_library_common(${URDF_FILE} ${ROOT_LINK} ${TIP_LINK} "cu")
endfunction()