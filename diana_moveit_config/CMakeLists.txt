cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)

project(diana_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Run the launch test under an isolated ROS_DOMAIN_ID so its move_group,
  # controller_manager and topic graph cannot collide with other packages'
  # ROS graph integration tests running in parallel.
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  find_package(ament_cmake_pytest REQUIRED)
  add_ros_isolated_launch_test(test/test_diana3_moveit_planning_smoke.py TIMEOUT 120)
  add_ros_isolated_launch_test(test/test_diana7g2_moveit_planning_smoke.py TIMEOUT 120)
endif()

ament_package()
