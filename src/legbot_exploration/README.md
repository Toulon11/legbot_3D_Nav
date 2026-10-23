# legbot_exploration

独立 ROS 2 Jazzy 功能包：同一功能分别由 `cpp/` 和 `python/` 实现。统一通过
`ament_cmake` 构建，并用 `ament_cmake_python` 安装 Python 模块；因此无需手写
`setup.py`。两个可执行文件为 `explorer_cpp` 和 `explorer_py`，一次只运行一个。

## 输入和输出

- 输入：统一适配器输出的 `/slam/global_cloud_registered`、
  `/slam/odometry_lidar`、`/slam/global_odometry_base`，以及 `/go2/demo_ready`。
  适配器可选择 Gazebo 真值、FAST-LIO2 或 FAST-LIVO2。若 SCAN 正在运行，还会读取
  `/local_planner/grid_map/occupancy_inflate` 过滤被规划器判为障碍的目标。
- 输出：`/exploration/frontiers` (MarkerArray)、`/exploration/status` (JSON String)，
  以及由机体里程计累计的 `/exploration/trajectory` (Path)。轨迹从探索节点启动时
  开始记录，每移动约 0.1 m 增加一点，RViz 中显示为绿色线。平层 RViz 固定
  `map` 视角概览整块场地，三种定位来源使用同一个视图。
- 自动导航时，目标发到 `/scan/waypoint_input`，由既有
  `scan_waypoint_mission` 以 `direct` 模式转发到 `/scan/goal`。节点根据转发目标的
  时间戳、`/scan/goal_stop_ack` 和 `/scan/mission_complete` 判断本次目标完成。
  若机器人已在 `goal_reach_xy` 内停留 `near_goal_dwell` 秒而 SCAN 未回执，
  探索节点先请求清除旧路点，成功后以 `goal_reached`、
  `detail=proximity_fallback` 记录到达，避免反复记为 `no_progress`。
- `/exploration/set_paused` 是 `std_srvs/SetBool`：`true` 暂停且清除自动目标，
  `false` 继续。手工在路点输入话题发目标时也会自动暂停探索。

## 运行

### 一条命令启动仿真探索

编译并执行 `source tools/env.sh` 后，运行：

```bash
ros2 launch legbot_exploration exploration_demo.launch.py
```

只在单层平地测试探索时，使用独立的 `ExplorationFlat.sdf` 场景：

```bash
ros2 launch legbot_exploration exploration_flat_demo.launch.py
```

这个场景沿用 GO2 的 URDF、雷达、相机和出生位置，保留平地障碍与边界墙，移除了多层 Building。`exploration_flat_demo.launch.py` 同样支持 `localization_source:=gazebo|fastlio|fastlivo` 和 `control_start_delay:=秒数`；原入口继续使用 Building 场景。

单层入口关闭 Gazebo、RViz 的参考网格，并在收到新鲜 SCAN 膨胀障碍图后才下发新目标；其目标避障半径为 0.50 m，单次目标距离上限为 4 m。探索范围按起始机体位姿设置为 `x∈[-4.5,26.5]`、`y∈[-21.5,9.5]` 米的相对偏移；使用默认 Gazebo 出生点 `(-27,6)` 时，对应围墙内的 `x∈[-31.5,-0.5]`、`y∈[-15.5,15.5]` 米。FAST-LIO2/FAST-LIVO2 在初始化时对齐到 Gazebo 全局坐标系，因此使用同一组探索边界。远处前沿先用已观测空闲格作中途目标，走近后再选下一点。修改启动文件或参数后需重新启动流程，运行中的节点不会自动换用新设置。

这个入口会启动 Gazebo、GO2 高层控制、SCAN、直达路点协调器、C++ 探索节点和
RViz。默认 `localization_source:=gazebo`，使用由 Gazebo 真值转换到 `map` 坐标系的
机体位姿、雷达位姿和点云，并自动下发探索目标。改用 FAST-LIO2 或 FAST-LIVO2 时，
在上述一键启动命令后分别添加 `localization_source:=fastlio` 或
`localization_source:=fastlivo`；一次只选一种定位来源。

`fastlio` 自动启动 FAST-LIO2 及其两级适配器；`fastlivo` 自动启动 FAST-LIVO2
及两级适配器，Gazebo 相机由仿真启动文件开启。两者的原始话题分别使用
`/fast_lio/*`、`/fast_livo/*`。三种来源对 GO2、SCAN、路点协调器和探索节点
统一发布 `/slam/global_odometry_base`、`/slam/odometry_lidar`、
`/slam/global_cloud_registered`。估计定位模式用同时间戳的 Gazebo 真值位姿仅做
初始化坐标对齐，后续位姿继续来自定位算法。对齐状态见 `/slam/alignment_status`。
不要再单独启动这些节点，以免出现重复发布者。

一键入口只开放 `localization_source` 和 `control_start_delay` 两个参数；默认
从启动流程开始计时，12 秒后启动 GO2 高层控制，可用
`control_start_delay:=20.0` 调整。需要观察模式或 Python 实现时，使用下方
`exploration.launch.py` 分终端启动。完整顺序见项目根目录的
`自主探索启动指南.md`。

### 分终端启动

先按根目录 `自主探索启动指南.md` 选择定位来源，并依次启动 Gazebo、定位、GO2、
SCAN，再启动本功能包。探索节点始终订阅统一的 `/slam/*` 话题，分终端启动时
无需再次指定 `localization_source`。默认只观察，不会发送导航目标；下面的命令
开启自动发点。
`exploration.launch.py` 默认同时启动一个直达路点协调器；
如果现有流程已启动该节点，请设 `start_waypoint_coordinator:=false`。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/likai/legbot_3D_Nav
colcon build --paths src/legbot_exploration
source install/setup.sh
ros2 launch legbot_exploration exploration.launch.py observation_only:=false
```

只想观察选点时，将 `observation_only:=false` 改为 `true`；需要 Python 实现时，
在同一命令末尾添加 `impl:=python`。默认实现为 C++，两个实现不要同时运行。

两个实现共用 `config/exploration.yaml`。地图逐帧用已配准点云及同时间戳的
雷达、机体位姿做射线更新：观察过的空闲体素、障碍体素和未观察体素分开保存。
体素的 Z 以**观测时的机体高度**为原点，故同层行走时定位的整体高度漂移不会
把早期观测从当前投影中排除。规划选点允许路径经过未知格，未知格不会因缺少
支撑而被判成障碍，符合当前 SCAN 策略。候选目标是 Frontier 簇里的实际未知格，
而非可能落在障碍内的几何质心。已观察的障碍和新近收到的 SCAN 膨胀障碍都会
参与筛选；活动目标若持续落入 SCAN 障碍，会提前撤销并暂时抑制。

高度归一化面向**同层探索**。跨层时相同 XY 的不同楼层可能重叠，需要独立的
楼层子图及楼层切换逻辑，不能直接把此模式用于跨层完成判定。若 SCAN 占据话题
不可用，节点仍可依靠自身地图选点，但会失去与 SCAN 障碍的一致性检查。

`exploration_complete` 表示平层入口在指定边界内持续没有未知前沿；
`current_region_complete` 只用于局部搜索模式，说明当前高度和搜索半径内持续没有 Frontier。
`frontiers_below_cluster_threshold` 表示仍有零散前沿，但尚未达到簇规模门槛；
单层全图模式持续这种状态 8 秒后，会按单格前沿再次尝试选点，目标仍须通过路径、距离和障碍筛选。
`frontiers_unreachable_or_suppressed` 表示仍有前沿簇，但暂时无法选用。
状态中的 `frontier_cells` 和 `frontier_clusters` 分别给出原始前沿格数和通过规模门槛的簇数。
`reachable_frontier_cells`、`candidate_goals`、`suppressed_goals`、`open_start_neighbors`
用于区分地图断连、目标冷却与起点周围被障碍膨胀包围。只有存在合格前沿簇、
却没有可达前沿格和候选目标时，全图模式才会从本次实际行驶轨迹里选择约 2–4 m
的已观察空闲回退点；状态的 `detail=recovery_backtrack` 表示正在执行这一步。
回退后若新增已观测 XY 格少于 `recovery_min_new_cells`，暂不重复回退；到达目标后
若观测增长不足，也暂不重复选择同一目标。地图新增足够格子后解除这两项抑制。
因此仍有前沿但都不可用时，节点会报告未解决的前沿状态，不会宣布探索完成。
`map_capacity_reached` 表示体素上限已到，需增大容量或重启探索。

当前版本适用于同层、平坦场景的首次联调。未知区按未占据处理，不具备悬崖或
无地面支撑自动停车能力。普通入口仍按 `local_search_radius` 做局部搜索；
平层入口启用有边界的全图搜索。`exploration_complete` 的含义是累计地图中
没有符合定义的未知前沿，不等同于逐平方米实地踏遍。

## 核心测试

```bash
PYTHONPATH=python python3 -m unittest discover -s test -p test_core.py
g++ -std=c++17 -Icpp/include cpp/src/exploration_core.cpp test/test_core.cpp -o /tmp/legbot_exploration_core_test
/tmp/legbot_exploration_core_test
```
