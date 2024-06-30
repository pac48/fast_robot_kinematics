function(generate_fast_forward_kinematics_library URDF_FILE ROOT_LINK TIP_LINK)

  find_package(Python REQUIRED COMPONENTS Interpreter)
  if (Python_Interpreter_FOUND)
    message(STATUS "Python executable: ${Python_EXECUTABLE}")
  else ()
    message(FATAL_ERROR "Python interpreter not found.")
  endif ()

  include(ExternalProject)
  ExternalProject_Add(
      LBFGSpp
      PREFIX ${CMAKE_BINARY_DIR}/LBFGSpp
      GIT_REPOSITORY https://github.com/yixuan/LBFGSpp.git
      GIT_TAG master
      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
  )
  ExternalProject_Get_Property(LBFGSpp source_dir)
  set(LBFGSppIncludeDir ${source_dir}/include)


  execute_process(
      COMMAND ${Python_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/robot_gen.py ${URDF_FILE} ${CMAKE_SOURCE_DIR}/scripts/robot_config.cpp.template ${CMAKE_CURRENT_BINARY_DIR}/forward_kinematics_lib.cpp ${ROOT_LINK} ${TIP_LINK}
      OUTPUT_VARIABLE FAST_FK_NUMBER_OF_JOINTS
      OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  add_custom_command(
      OUTPUT forward_kinematics_lib.cpp
      COMMAND ${Python_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/robot_gen.py ${URDF_FILE} ${CMAKE_SOURCE_DIR}/scripts/robot_config.cpp.template
      ${CMAKE_CURRENT_BINARY_DIR}/forward_kinematics_lib.cpp ${ROOT_LINK} ${TIP_LINK}
      DEPENDS ${URDF_FILE} ${CMAKE_SOURCE_DIR}/scripts/robot_config.cpp.template # FAST_FK_NUMBER_OF_JOINTS
      COMMENT
      "Running `${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/robot_gen.py ${URDF_FILE} ${CMAKE_SOURCE_DIR}/scripts/robot_config.cpp.template
    ${CMAKE_CURRENT_BINARY_DIR}/forward_kinematics_test.cpp ${ROOT_LINK} ${TIP_LINK}`"
      VERBATIM
  )

  add_library(fast_forward_kinematics_library SHARED forward_kinematics_lib.cpp)
  target_include_directories(fast_forward_kinematics_library PUBLIC ${CMAKE_SOURCE_DIR}/include)
  target_compile_definitions(fast_forward_kinematics_library PUBLIC "${FAST_FK_NUMBER_OF_JOINTS}")
  set_target_properties(fast_forward_kinematics_library PROPERTIES CMAKE_BUILD_TYPE Release)

  option(USE_EIGEN "Enables Eigen types in the generated library" ON)
  find_package(Eigen3 3.3 NO_MODULE)
  if(Eigen3_FOUND AND USE_EIGEN)
    add_dependencies(fast_forward_kinematics_library LBFGSpp)
    target_link_libraries(fast_forward_kinematics_library PUBLIC Eigen3::Eigen)
    target_include_directories(fast_forward_kinematics_library PUBLIC ${LBFGSppIncludeDir})
    target_compile_definitions(fast_forward_kinematics_library PUBLIC FAST_FK_USE_EIGEN)
  endif ()


endfunction()