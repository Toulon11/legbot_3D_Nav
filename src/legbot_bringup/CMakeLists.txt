cmake_minimum_required(VERSION 3.16)
project(legbot_bringup)
find_package(ament_cmake REQUIRED)
find_package(legbot_runtime REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
foreach(n reference_path_transform fastlio_odom_adapter cloud_transform)
add_executable(${n} src/${n}.cpp)
target_compile_features(${n} PUBLIC cxx_std_17)
ament_target_dependencies(${n} legbot_runtime tf2_sensor_msgs)
install(TARGETS ${n} DESTINATION lib/${PROJECT_NAME})
endforeach()
add_executable(global_navigation_adapter src/global_navigation_adapter.cpp)
target_compile_features(global_navigation_adapter PUBLIC cxx_std_17)
ament_target_dependencies(global_navigation_adapter
  builtin_interfaces rclcpp nav_msgs sensor_msgs std_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs)
install(TARGETS global_navigation_adapter DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/go2_command_adapter
  scripts/go2_estop_keyboard
  scripts/go2_demo_sequencer
  scripts/pct_path_mission
  scripts/fastlio_truth_monitor
  scripts/scan_waypoint_mission
  scripts/stair_cloud_sim
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config models worlds rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
ament_package()
