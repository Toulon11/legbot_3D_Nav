# 声明构建此包需要的最低 CMake 版本。
cmake_minimum_required(VERSION 3.16)
# 设置本功能包的构建项目名称。
project(legbot_exploration)

# 查找提供 ROS 2 包构建命令的 ament_cmake。
find_package(ament_cmake REQUIRED)
# 查找用于安装 Python 模块的构建扩展。
find_package(ament_cmake_python REQUIRED)
# 查找 C++ 版 ROS 2 节点库。
find_package(rclcpp REQUIRED)
# 目标位姿 PoseStamped 定义在 geometry_msgs 中。
find_package(geometry_msgs REQUIRED)
# 里程计与轨迹消息定义在 nav_msgs 中。
find_package(nav_msgs REQUIRED)
# 点云消息定义在 sensor_msgs 中。
find_package(sensor_msgs REQUIRED)
# 布尔就绪信号和状态字符串定义在 std_msgs 中。
find_package(std_msgs REQUIRED)
# 探索暂停服务使用 std_srvs 类型。
find_package(std_srvs REQUIRED)
# RViz 前沿标记使用 visualization_msgs 类型。
find_package(visualization_msgs REQUIRED)
find_package(scan_planner REQUIRED)

# 将单层探索节点、建图选点、路径规划和路径协调编成 explorer_cpp。
add_executable(explorer_cpp cpp/src/single_floor_explorer_node.cpp cpp/src/single_floor_exploration.cpp
  cpp/src/same_floor_route_planner.cpp cpp/src/route_orchestrator.cpp
  cpp/src/recovery_planner.cpp)
# 让编译器能找到 cpp/include/ 下的 single_floor_exploration.hpp。
target_include_directories(explorer_cpp PRIVATE cpp/include)
# C++ 代码用到 C++17 的 optional 等特性。
target_compile_features(explorer_cpp PRIVATE cxx_std_17)
# 把上方 ROS 依赖的头文件与库链接到 explorer_cpp。
ament_target_dependencies(explorer_cpp rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs visualization_msgs scan_planner)
# 安装为 ros2 run legbot_exploration explorer_cpp 能找到的位置。
install(TARGETS explorer_cpp DESTINATION lib/${PROJECT_NAME})

# A separate observational map for multi-level terrain. It never sends goals
# and does not alter the C++ single-floor explorer.
add_executable(online_layer_map_cpp
  cpp/src/online_layer_map_node.cpp cpp/src/multi_layer_map.cpp
  cpp/src/surface_graph.cpp)
target_include_directories(online_layer_map_cpp PRIVATE cpp/include)
target_compile_features(online_layer_map_cpp PRIVATE cxx_std_17)
ament_target_dependencies(online_layer_map_cpp rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs visualization_msgs)
install(TARGETS online_layer_map_cpp DESTINATION lib/${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  add_executable(test_exploration_core
    test/test_single_floor_exploration.cpp cpp/src/single_floor_exploration.cpp)
  target_include_directories(test_exploration_core PRIVATE cpp/include)
  target_compile_features(test_exploration_core PRIVATE cxx_std_17)
  target_compile_options(test_exploration_core PRIVATE -UNDEBUG)
  add_test(NAME exploration_core_frontiers COMMAND test_exploration_core)
  add_executable(test_multi_layer_map
    test/test_multi_layer_map.cpp cpp/src/multi_layer_map.cpp)
  target_include_directories(test_multi_layer_map PRIVATE cpp/include)
  target_compile_features(test_multi_layer_map PRIVATE cxx_std_17)
  target_compile_options(test_multi_layer_map PRIVATE -UNDEBUG)
  add_test(NAME online_multi_layer_map_geometry COMMAND test_multi_layer_map)
  add_executable(test_surface_graph
    test/test_surface_graph.cpp cpp/src/surface_graph.cpp)
  target_include_directories(test_surface_graph PRIVATE cpp/include)
  target_compile_features(test_surface_graph PRIVATE cxx_std_17)
  target_compile_options(test_surface_graph PRIVATE -UNDEBUG)
  add_test(NAME online_surface_graph_astar COMMAND test_surface_graph)
  add_executable(test_route_planner
    test/test_same_floor_route_planner.cpp cpp/src/same_floor_route_planner.cpp
    cpp/src/recovery_planner.cpp cpp/src/single_floor_exploration.cpp)
  target_include_directories(test_route_planner PRIVATE cpp/include)
  target_compile_features(test_route_planner PRIVATE cxx_std_17)
  target_compile_options(test_route_planner PRIVATE -UNDEBUG)
  add_test(NAME exploration_route_safety COMMAND test_route_planner)
  add_executable(test_planning_pipeline
    test/test_route_orchestrator.cpp cpp/src/route_orchestrator.cpp
    cpp/src/same_floor_route_planner.cpp cpp/src/recovery_planner.cpp
    cpp/src/single_floor_exploration.cpp)
  target_include_directories(test_planning_pipeline PRIVATE cpp/include)
  target_compile_features(test_planning_pipeline PRIVATE cxx_std_17)
  target_compile_options(test_planning_pipeline PRIVATE -UNDEBUG)
  add_test(NAME exploration_planning_pipeline COMMAND test_planning_pipeline)
endif()

# 把 python/legbot_exploration/ 安装为可导入的 Python 模块。
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})
install(PROGRAMS python/scripts/verify_flat_coverage.py DESTINATION lib/${PROJECT_NAME})
# 安装参数 YAML 与 launch 文件；排除 Python 自动生成的缓存文件。
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
# 完成 ament 包注册，让其他 ROS 2 包可以查找本包。
ament_package()
