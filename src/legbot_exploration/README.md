# legbot_exploration

独立 ROS 2 Jazzy 功能包：同一功能分别由 `cpp/` 和 `python/` 实现。统一通过
`ament_cmake` 构建，并用 `ament_cmake_python` 安装 Python 模块；因此无需手写
`setup.py`。两个可执行文件为 `explorer_cpp` 和 `explorer_py`，一次只运行一个。

## 输入和输出

- 输入：`/fast_lio/cloud_registered`、`/fast_lio/odometry_lidar`、
  `/fast_lio/odometry_base`、`/go2/demo_ready`；若 SCAN 正在运行，还会读取
  `/local_planner/grid_map/occupancy_inflate` 过滤被规划器判为障碍的目标。
- 输出：`/exploration/frontiers` (MarkerArray)、`/exploration/status` (JSON String)。
- 自动导航时，目标发到 `/scan/waypoint_input`，由既有
  `scan_waypoint_mission` 以 `direct` 模式转发到 `/scan/goal`。节点根据转发目标的
  时间戳、`/scan/goal_stop_ack` 和 `/scan/mission_complete` 判断本次目标完成。
  若机器人已在 `goal_reach_xy` 内停留 `near_goal_dwell` 秒而 SCAN 未回执，
  探索节点先请求清除旧路点，成功后以 `goal_reached`、
  `detail=proximity_fallback` 记录到达，避免反复记为 `no_progress`。
- `/exploration/set_paused` 是 `std_srvs/SetBool`：`true` 暂停且清除自动目标，
  `false` 继续。手工在路点输入话题发目标时也会自动暂停探索。

## 运行

先启动现有的 FAST-LIO、SCAN、GO2 协调流程，再启动本功能包。默认只观察，
不会发送导航目标。`exploration.launch.py` 默认同时启动一个直达路点协调器；
如果现有流程已启动该节点，请设 `start_waypoint_coordinator:=false`。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/likai/legbot_3D_Nav
colcon build --paths src/legbot_exploration
source install/setup.sh
ros2 launch legbot_exploration exploration.launch.py impl:=cpp observation_only:=true
```

观察状态和 RViz Frontier 标记后，在平坦仿真场景启用自动发点：

```bash
ros2 launch legbot_exploration exploration.launch.py impl:=python observation_only:=false
```

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

`current_region_complete` 只说明当前高度和搜索半径内持续没有 Frontier。
`frontiers_unreachable_or_suppressed` 表示仍有 Frontier，但暂时无法选用。
`map_capacity_reached` 表示体素上限已到，需增大容量或重启探索。

当前版本适用于同层、平坦场景的首次联调。未知区按未占据处理，不具备悬崖或
无地面支撑自动停车能力。`current_region_complete` 也不代表整张 64×32 m 地面
已经探索完毕：当前仅搜索机体周围 `local_search_radius` 范围，并限制目标距离
不超过 `max_goal_distance`。全局覆盖需要后续增加全图前沿搜索和远处前沿的
分段引导。

## 核心测试

```bash
PYTHONPATH=python python3 -m unittest discover -s test -p test_core.py
g++ -std=c++17 -Icpp/include cpp/src/exploration_core.cpp test/test_core.cpp -o /tmp/legbot_exploration_core_test
/tmp/legbot_exploration_core_test
```
