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
# 暂停与清除服务使用 std_srvs 类型。
find_package(std_srvs REQUIRED)
# RViz 前沿标记使用 visualization_msgs 类型。
find_package(visualization_msgs REQUIRED)

# 将 ROS 节点与核心算法两个 .cpp 文件编成 explorer_cpp 可执行程序。
add_executable(explorer_cpp cpp/src/explorer_node.cpp cpp/src/exploration_core.cpp)
# 让编译器能找到 cpp/include/ 下的 exploration_core.hpp。
target_include_directories(explorer_cpp PRIVATE cpp/include)
# C++ 代码用到 C++17 的 optional 等特性。
target_compile_features(explorer_cpp PRIVATE cxx_std_17)
# 把上方 ROS 依赖的头文件与库链接到 explorer_cpp。
ament_target_dependencies(explorer_cpp rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs visualization_msgs)
# 安装为 ros2 run legbot_exploration explorer_cpp 能找到的位置。
install(TARGETS explorer_cpp DESTINATION lib/${PROJECT_NAME})

# 把 python/legbot_exploration/ 安装为可导入的 Python 模块。
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})
# 安装 Python 可执行入口，供 ros2 run 或 launch 调用。
install(PROGRAMS python/scripts/explorer_py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS python/scripts/verify_flat_coverage.py DESTINATION lib/${PROJECT_NAME})
# 安装参数 YAML 与 launch 文件；排除 Python 自动生成的缓存文件。
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
# 完成 ament 包注册，让其他 ROS 2 包可以查找本包。
ament_package()
