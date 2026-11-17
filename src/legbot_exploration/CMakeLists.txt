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
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# 点云消息定义在 sensor_msgs 中。
find_package(sensor_msgs REQUIRED)
# 布尔就绪信号和状态字符串定义在 std_msgs 中。
find_package(std_msgs REQUIRED)
# 探索暂停服务使用 std_srvs 类型。
find_package(std_srvs REQUIRED)
# RViz 前沿标记使用 visualization_msgs 类型。
find_package(visualization_msgs REQUIRED)
find_package(scan_planner REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
pybind11_add_module(_legbot_layer_kernels cpp/src/layer_kernels.cpp)
target_compile_features(_legbot_layer_kernels PRIVATE cxx_std_17)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS _legbot_layer_kernels LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}")
add_library(exploration_pct_backend STATIC cpp/src/pct_backend.cpp)
target_include_directories(exploration_pct_backend PUBLIC cpp/include)
target_compile_features(exploration_pct_backend PUBLIC cxx_std_17)
target_compile_definitions(exploration_pct_backend PRIVATE
  EXPLORATION_PYTHON_DIR="${CMAKE_CURRENT_SOURCE_DIR}/python")
target_link_libraries(exploration_pct_backend PUBLIC Python3::Python)

# One executable owns all startup modes. PCT owns mapping, inspection and
# native A* ascent preview; no legacy surface/ray graph is compiled.
add_executable(combined_exploration_cpp
  cpp/src/combined_exploration_main.cpp
  cpp/src/single_floor_explorer_node.cpp cpp/src/single_floor_exploration.cpp
  cpp/src/exploration_controller.cpp cpp/src/exploration_policy.cpp
  cpp/src/exploration_recovery.cpp cpp/src/exploration_route_dispatch.cpp
  cpp/src/exploration_candidates.cpp cpp/src/point_safety.cpp cpp/src/same_floor_route_planner.cpp
  cpp/src/route_orchestrator.cpp cpp/src/recovery_request_planner.cpp
  cpp/src/online_layer_map_node.cpp)
target_include_directories(combined_exploration_cpp PRIVATE cpp/include)
target_compile_features(combined_exploration_cpp PRIVATE cxx_std_17)
target_link_libraries(combined_exploration_cpp exploration_pct_backend)
ament_target_dependencies(combined_exploration_cpp rclcpp geometry_msgs nav_msgs
  sensor_msgs std_msgs std_srvs visualization_msgs scan_planner ament_index_cpp control_msgs tf2 tf2_ros)
install(TARGETS combined_exploration_cpp DESTINATION lib/${PROJECT_NAME})
# Preserve external ros2 run/benchmark names without compiling duplicate nodes.
# The single main selects the mode from the invoked executable name.
install(CODE [[
  foreach(name explorer_cpp online_layer_map_cpp)
    set(link "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/legbot_exploration/${name}")
    file(REMOVE "${link}")
    file(CREATE_LINK combined_exploration_cpp "${link}" SYMBOLIC)
  endforeach()
]])

include(CTest)
if(BUILD_TESTING)
  add_executable(test_exploration_core
    test/test_single_floor_exploration.cpp cpp/src/single_floor_exploration.cpp)
  target_include_directories(test_exploration_core PRIVATE cpp/include)
  target_compile_features(test_exploration_core PRIVATE cxx_std_17)
  target_compile_options(test_exploration_core PRIVATE -UNDEBUG)
  add_test(NAME exploration_core_frontiers COMMAND test_exploration_core)
  add_executable(test_route_planner
    test/test_same_floor_route_planner.cpp cpp/src/exploration_candidates.cpp cpp/src/point_safety.cpp cpp/src/same_floor_route_planner.cpp
    cpp/src/single_floor_exploration.cpp)
  target_include_directories(test_route_planner PRIVATE cpp/include)
  target_compile_features(test_route_planner PRIVATE cxx_std_17)
  target_compile_options(test_route_planner PRIVATE -UNDEBUG)
  add_test(NAME exploration_route_safety COMMAND test_route_planner)
  add_executable(test_planning_pipeline
    test/test_route_orchestrator.cpp cpp/src/route_orchestrator.cpp cpp/src/recovery_request_planner.cpp
    cpp/src/exploration_candidates.cpp cpp/src/point_safety.cpp cpp/src/same_floor_route_planner.cpp
    cpp/src/single_floor_exploration.cpp)
  target_link_libraries(test_planning_pipeline exploration_pct_backend)
  target_include_directories(test_planning_pipeline PRIVATE cpp/include)
  target_compile_features(test_planning_pipeline PRIVATE cxx_std_17)
  target_compile_options(test_planning_pipeline PRIVATE -UNDEBUG)
  add_test(NAME exploration_planning_pipeline COMMAND test_planning_pipeline
    ${CMAKE_CURRENT_SOURCE_DIR}/../../install/pct_planner/share/pct_planner)
  add_executable(test_pct_embedding test/test_pct_embedding.cpp)
  target_link_libraries(test_pct_embedding exploration_pct_backend)
  target_compile_options(test_pct_embedding PRIVATE -UNDEBUG)
  add_test(NAME exploration_pct_embedding COMMAND test_pct_embedding
    ${CMAKE_CURRENT_SOURCE_DIR}/../../install/pct_planner/share/pct_planner)
  set_tests_properties(exploration_pct_embedding PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME exploration_fixed_layer_map COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_fixed_layer_map.py)
  add_test(NAME exploration_pct_adapter COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_online_pct.py)
  add_test(NAME exploration_surface_observation COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_surface_observation.py)
  set_tests_properties(exploration_surface_observation PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:$ENV{PYTHONPATH}")
  add_test(NAME exploration_pct_native_accessor COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_native_accessor.py)
  add_test(NAME exploration_pct_controller COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp>)
  add_test(NAME exploration_pct_observation_ros COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --observation)
  set_tests_properties(exploration_pct_observation_ros PROPERTIES TIMEOUT 45 RUN_SERIAL TRUE
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/../../.pct_runtime:$ENV{PYTHONPATH};AMENT_PREFIX_PATH=${CMAKE_CURRENT_SOURCE_DIR}/../../install/pct_planner:$ENV{AMENT_PREFIX_PATH}")
  add_test(NAME exploration_pct_completion COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --completion)
  add_test(NAME exploration_pct_recovery COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --recovery)
  add_test(NAME exploration_pct_recovery_occupied COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --recovery-occupied)
  add_test(NAME exploration_pct_startup_low_body COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --startup-low-body)
  set_tests_properties(exploration_pct_startup_low_body PROPERTIES TIMEOUT 55 RUN_SERIAL TRUE
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/../../.pct_runtime:$ENV{PYTHONPATH};AMENT_PREFIX_PATH=${CMAKE_CURRENT_SOURCE_DIR}/../../install/pct_planner:$ENV{AMENT_PREFIX_PATH}")
  add_test(NAME exploration_pct_multilevel_support COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --multilevel-support)
  set_tests_properties(exploration_pct_multilevel_support PROPERTIES TIMEOUT 40 RUN_SERIAL TRUE
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/../../.pct_runtime:$ENV{PYTHONPATH};AMENT_PREFIX_PATH=${CMAKE_CURRENT_SOURCE_DIR}/../../install/pct_planner:$ENV{AMENT_PREFIX_PATH}")
  set_tests_properties(exploration_pct_recovery_occupied PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 45 RUN_SERIAL TRUE)
  add_test(NAME exploration_pct_local_global COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --local-global)
  add_test(NAME exploration_pct_local_pending COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --local-pending)
  add_test(NAME exploration_pct_cross_layer COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --cross-layer)
  add_test(NAME exploration_pct_floor_exploration COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --floor-exploration)
  add_test(NAME exploration_pct_fast_floor COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --fast-floor)
  add_test(NAME exploration_pct_fast_floor_observation COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --fast-floor-observation)
  add_test(NAME exploration_pct_floor_skip COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --floor-skip)
  add_test(NAME exploration_pct_short_floor COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --short-floor)
  add_test(NAME exploration_pct_stair_frontier_scout COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --stair-frontier-scout)
  add_test(NAME exploration_pct_stair_priority COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --stair-priority)
  add_test(NAME exploration_pct_local_stair_probe COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --local-stair-probe)
  add_test(NAME exploration_pct_floor_ascent_handoff COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --floor-ascent-handoff)
  add_test(NAME exploration_pct_floor_descent_arrival COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --floor-descent-arrival)
  set_tests_properties(exploration_pct_floor_ascent_handoff exploration_pct_floor_descent_arrival PROPERTIES TIMEOUT 55 RUN_SERIAL TRUE
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/../../.pct_runtime:$ENV{PYTHONPATH};AMENT_PREFIX_PATH=${CMAKE_CURRENT_SOURCE_DIR}/../../install/pct_planner:$ENV{AMENT_PREFIX_PATH}")
  add_test(NAME exploration_pct_stair_reverse COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --stair-reverse)
  add_test(NAME exploration_pct_stair_reverse_partial COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --stair-reverse-partial)
  add_test(NAME exploration_pct_stair_gap_scan COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --stair-gap-scan)
  add_test(NAME exploration_pct_flat_gap_scan COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --flat-gap-scan)
  add_test(NAME exploration_pct_landing_shift COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --landing-shift)
  set_tests_properties(exploration_pct_stair_gap_scan exploration_pct_flat_gap_scan exploration_pct_landing_shift exploration_pct_stair_priority exploration_pct_local_stair_probe PROPERTIES TIMEOUT 50 RUN_SERIAL TRUE
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/../../.pct_runtime:$ENV{PYTHONPATH};AMENT_PREFIX_PATH=${CMAKE_CURRENT_SOURCE_DIR}/../../install/pct_planner:$ENV{AMENT_PREFIX_PATH}")
  add_test(NAME exploration_pct_floor_deferred COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --floor-deferred)
  add_test(NAME exploration_pct_floor_blacklist COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --floor-blacklist)
  add_test(NAME exploration_pct_floor_no_observation COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --floor-no-observation)
  add_test(NAME exploration_pct_ordinary_floor_trial COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --ordinary-floor-trial)
  add_test(NAME exploration_pct_bounded_floor_review COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --bounded-floor-review)
  set_tests_properties(exploration_pct_bounded_floor_review PROPERTIES TIMEOUT 45 RUN_SERIAL TRUE
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/../../.pct_runtime:$ENV{PYTHONPATH};AMENT_PREFIX_PATH=${CMAKE_CURRENT_SOURCE_DIR}/../../install/pct_planner:$ENV{AMENT_PREFIX_PATH}")
  add_test(NAME exploration_pct_floor_disconnected COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --floor-disconnected)
  add_test(NAME exploration_pct_floor_blind_observers COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_controller_ros.py $<TARGET_FILE:combined_exploration_cpp> --floor-blind-observers)
  set_tests_properties(exploration_pct_floor_blind_observers PROPERTIES TIMEOUT 50 RUN_SERIAL TRUE
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/../../.pct_runtime:$ENV{PYTHONPATH};AMENT_PREFIX_PATH=${CMAKE_CURRENT_SOURCE_DIR}/../../install/pct_planner:$ENV{AMENT_PREFIX_PATH}")
  add_test(NAME exploration_pct_map_only COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_map_ros.py $<TARGET_FILE:combined_exploration_cpp>)
  add_test(NAME exploration_pct_cloud_sources COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_cloud_sources_ros.py $<TARGET_FILE:combined_exploration_cpp>)
  set_tests_properties(exploration_pct_controller exploration_pct_completion exploration_pct_recovery exploration_pct_local_global exploration_pct_local_pending exploration_pct_cross_layer exploration_pct_floor_exploration exploration_pct_fast_floor exploration_pct_fast_floor_observation exploration_pct_floor_skip exploration_pct_short_floor exploration_pct_stair_frontier_scout exploration_pct_stair_reverse exploration_pct_stair_reverse_partial exploration_pct_floor_deferred exploration_pct_floor_blacklist exploration_pct_floor_no_observation exploration_pct_floor_disconnected exploration_pct_map_only exploration_pct_cloud_sources PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 45 RUN_SERIAL TRUE
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/../../.pct_runtime:$ENV{PYTHONPATH};AMENT_PREFIX_PATH=${CMAKE_CURRENT_SOURCE_DIR}/../../install/pct_planner:$ENV{AMENT_PREFIX_PATH}")
  add_test(NAME exploration_execution_actions COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_execution_actions_ros.py)
  # Climb arrivals now execute all four scan views before the next waypoint.
  set_tests_properties(exploration_pct_floor_skip exploration_pct_floor_exploration
    exploration_pct_short_floor exploration_pct_stair_frontier_scout PROPERTIES TIMEOUT 90)
  set_tests_properties(exploration_execution_actions PROPERTIES TIMEOUT 45 RUN_SERIAL TRUE)
  add_test(NAME exploration_pct_frontiers COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_frontiers.py)
  add_test(NAME exploration_pct_observation COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_observation.py)
  set_tests_properties(exploration_pct_observation PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:$ENV{PYTHONPATH}")
  add_test(NAME exploration_pct_frontier_index COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_frontier_index.py)
  add_test(NAME exploration_frontier_component_updates COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_frontier_component_updates.py)
  set_tests_properties(exploration_frontier_component_updates PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:$ENV{PYTHONPATH}")
  add_test(NAME exploration_parallel_slices COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_parallel_slices.py)
  set_tests_properties(exploration_parallel_slices PROPERTIES
    ENVIRONMENT "PYTHONPATH=$<TARGET_FILE_DIR:_legbot_layer_kernels>:${CMAKE_CURRENT_SOURCE_DIR}/python:$ENV{PYTHONPATH}")
  add_test(NAME exploration_pct_local_window COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_local_window.py)
  set_tests_properties(exploration_pct_local_window PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:$ENV{PYTHONPATH}")
  add_test(NAME exploration_pct_connectivity COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pct_connectivity.py)
  set_tests_properties(exploration_pct_connectivity PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:$ENV{PYTHONPATH}")
  set_tests_properties(exploration_pct_frontier_index PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:$ENV{PYTHONPATH}")
  set_tests_properties(exploration_pct_frontiers PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/../../.pct_runtime:$ENV{PYTHONPATH}")
  add_test(NAME exploration_pct_gpu_equivalence COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/verify_pct_gpu.py)
  set_tests_properties(exploration_pct_gpu_equivalence PROPERTIES SKIP_RETURN_CODE 77
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/../../.pct_runtime:$ENV{PYTHONPATH}")
  set_tests_properties(exploration_pct_adapter exploration_fixed_layer_map exploration_pct_native_accessor PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/../../.pct_runtime:$ENV{PYTHONPATH}")
  set_property(TEST exploration_pct_adapter exploration_fixed_layer_map exploration_pct_native_accessor
    exploration_pct_frontiers exploration_pct_observation exploration_pct_frontier_index exploration_frontier_component_updates
    exploration_pct_local_window exploration_pct_connectivity exploration_pct_gpu_equivalence exploration_parallel_slices
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
    "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/../pct_planner/planner/scripts")
endif()

# 把 python/legbot_exploration/ 安装为可导入的 Python 模块。
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})
install(PROGRAMS python/scripts/verify_flat_coverage.py DESTINATION lib/${PROJECT_NAME})
# 安装参数 YAML 与 launch 文件；排除 Python 自动生成的缓存文件。
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES requirements-pct.txt DESTINATION share/${PROJECT_NAME})
# 完成 ament 包注册，让其他 ROS 2 包可以查找本包。
ament_package()
