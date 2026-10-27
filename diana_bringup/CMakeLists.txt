cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)

project(diana_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  ament_lint_auto_find_test_dependencies()

  if(NOT ROBOT_IP)
    set(ROBOT_IP "172.16.0.1")
    message(STATUS "ROBOT_IP not defined, defaulting to ${ROBOT_IP}...")
  endif()

  set(SUPPORTED_ROBOT_TYPES diana3 diana7 diana7g2)
  if(NOT ROBOT_TYPE)
    set(ROBOT_TYPE "diana3")
    message(STATUS "ROBOT_TYPE not defined, defaulting to ${ROBOT_TYPE}...")
  endif()
  if(NOT ROBOT_TYPE IN_LIST SUPPORTED_ROBOT_TYPES)
    list(JOIN SUPPORTED_ROBOT_TYPES ", " SUPPORTED_ROBOT_TYPES_STRING)
    message(FATAL_ERROR
      "ROBOT_TYPE '${ROBOT_TYPE}' is not supported, expected one of: ${SUPPORTED_ROBOT_TYPES_STRING}")
  endif()

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    if(ARGN)
      add_launch_test("${path}" RUNNER "${RUNNER}" ARGS ${ARGN} TIMEOUT 500)
    else()
      add_launch_test("${path}" RUNNER "${RUNNER}" TIMEOUT 500)
    endif()
  endfunction()

  add_ros_isolated_launch_test(test/test_hardware_example_controllers.py
    "robot_ip:=${ROBOT_IP}" "robot_type:=${ROBOT_TYPE}")
  add_ros_isolated_launch_test(test/test_hardware_generic_controller.py
    "robot_ip:=${ROBOT_IP}" "robot_type:=${ROBOT_TYPE}")
  # Fake-hardware (no-robot) integration tests.
  add_ros_isolated_launch_test(test/test_fake_hardware_controllers.py)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_diana_arm_interfaces test/test_diana_arm_interfaces.py)
  ament_add_pytest_test(test_xacro_path_resolution test/test_xacro_path_resolution.py)
  ament_add_pytest_test(test_gazebo_wrapper_expansion test/test_gazebo_wrapper_expansion.py)
  ament_add_pytest_test(test_launch_file_parsing test/test_launch_file_parsing.py)

  install(
    DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
