function(generate_fast_forward_kinematics_library URDF_FILE ROOT_LINK TIP_LINK)

  find_package(Python REQUIRED COMPONENTS Interpreter)
  if (Python_Interpreter_FOUND)
    message(STATUS "Python executable: ${Python_EXECUTABLE}")
  else ()
    message(FATAL_ERROR "Python interpreter not found.")
  endif ()

  execute_process(
      COMMAND ${Python_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/robot_gen.py ${URDF_FILE} ${CMAKE_SOURCE_DIR}/scripts/robot_config.cpp.template ${CMAKE_CURRENT_BINARY_DIR}/forward_kinematics_test.cpp ${ROOT_LINK} ${TIP_LINK}
      OUTPUT_VARIABLE FAST_FK_NUMBER_OF_JOINTS
      OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  add_custom_command(
      OUTPUT forward_kinematics_test.cpp
      COMMAND ${Python_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/robot_gen.py ${URDF_FILE} ${CMAKE_SOURCE_DIR}/scripts/robot_config.cpp.template
      ${CMAKE_CURRENT_BINARY_DIR}/forward_kinematics_test.cpp ${ROOT_LINK} ${TIP_LINK}
      DEPENDS ${URDF_FILE} ${CMAKE_SOURCE_DIR}/scripts/robot_config.cpp.template # FAST_FK_NUMBER_OF_JOINTS
      COMMENT
      "Running `${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/robot_gen.py ${URDF_FILE} ${CMAKE_SOURCE_DIR}/scripts/robot_config.cpp.template
    ${CMAKE_CURRENT_BINARY_DIR}/forward_kinematics_test.cpp ${ROOT_LINK} ${TIP_LINK}`"
      VERBATIM
  )

  add_library(fast_forward_kinematics_library SHARED forward_kinematics_test.cpp)
  target_include_directories(fast_forward_kinematics_library PUBLIC ${CMAKE_SOURCE_DIR}/include)
  target_compile_definitions(fast_forward_kinematics_library PUBLIC "${FAST_FK_NUMBER_OF_JOINTS}")
  set_target_properties(fast_forward_kinematics_library PROPERTIES CMAKE_BUILD_TYPE Release)
  target_compile_options(fast_forward_kinematics_library PUBLIC -O3 -march=native)

endfunction()