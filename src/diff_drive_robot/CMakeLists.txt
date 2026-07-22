cmake_minimum_required(VERSION 3.8)
project(diff_drive_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/navigation.py
    scripts/pid_controller.py
    scripts/llm_nav.py
    scripts/bt_server.py
    scripts/path_planning.py
    scripts/waypoint_nav.py
    scripts/check_odometry.py
    scripts/reset_pose.py
    scripts/odom_tf_broadcaster.py
    scripts/tf_static_relay.py
    scripts/tf_map_relay.py
    scripts/collision_monitor.py
    scripts/mission_server.py
    scripts/obstacle_tracker.py
    scripts/priority_collision_avoidance.py
    scripts/keyboard_teleop.py
    scripts/deadlock_recovery.py
    scripts/frontier_explorer.py
    scripts/frontier_coordinator.py
    scripts/coverage_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf config worlds rviz maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()