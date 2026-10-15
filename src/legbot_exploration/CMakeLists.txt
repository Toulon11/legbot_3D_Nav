cmake_minimum_required(VERSION 3.16)
project(legbot_exploration)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(explorer_cpp cpp/src/explorer_node.cpp cpp/src/exploration_core.cpp)
target_include_directories(explorer_cpp PRIVATE cpp/include)
target_compile_features(explorer_cpp PRIVATE cxx_std_17)
ament_target_dependencies(explorer_cpp rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs visualization_msgs)
install(TARGETS explorer_cpp DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})
install(PROGRAMS python/scripts/explorer_py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
ament_package()
