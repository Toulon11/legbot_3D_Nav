# legbot_exploration

第一次阅读源码可先看 [代码阅读指南](代码阅读指南.md)：它解释 ROS 话题、体素地图、前沿选点、状态机和 C++/Python 对应关系；关键语句与参数旁也已补上中文注释。

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

## 探索方案、公式与策略

本包按“配准点云与位姿 → 累计三维证据 → 同层二维投影 → 前沿聚类与选点 →
SCAN 执行 → 根据新观测重新选点”的闭环运行。C++ 的 `exploration_core.cpp` 和
Python 的 `exploration_core.py` 使用相同的核心规则；定位来源由上游适配器选择，
本包只读取统一的 `/slam/*` 接口。以下公式描述的是**当前代码**，其中 $r$ 为网格分辨率，
默认 $r=0.25\,\mathrm m$；位置与点云都处于统一的 `map` 坐标系。

### 1. 点云同步与累计地图

每帧点云选取时间戳最接近的雷达、机体里程计；两者与点云的时间差都不能超过
`sensor_sync_tolerance`（默认 $0.03\,\mathrm s$）。点云先按 `point_stride` 抽稀，
并受 `max_points_per_cloud` 限制。以雷达位姿为射线起点，逐点沿射线更新稀疏体素表。
为降低同层行走时整体高度漂移的影响，体素高度使用采集该帧时的机体高度 $z_b(t)$：

$$
v=\left(\left\lfloor\frac{x}{r}\right\rfloor,
         \left\lfloor\frac{y}{r}\right\rfloor,
         \left\lfloor\frac{z-z_b(t)}{r}\right\rfloor\right).
$$

对每次射线经过或命中的体素 $v$，整数证据 $E(v)$ 按下式更新：

$$
E_{k+1}(v)=\operatorname{clip}\bigl(E_k(v)+\delta_k(v),-3,6\bigr),
\qquad
\delta_k(v)=
\begin{cases}
+3,&\text{射线终点位于配置的障碍高度区间；}\\
-1,&\text{射线经过该格，或终点不属于障碍。}
\end{cases}
$$

这里 $k$ 是**体素被更新的次数**，不是点云帧号。同一格可被多条射线反复更新。
表中没有记录的体素是未知；$E(v)>0$ 提供障碍证据，$E(v)\leq 0$ 提供已观察空闲证据。
二维投影只取配置的机体相对高度区间：同一水平格只要有符合障碍高度条件的正证据，
就记为障碍；否则若有非正证据，记为已知空闲；两种证据都没有则保持未知。
投影结果可记作 $M(q)\in\{\mathrm{free},\mathrm{occupied},\mathrm{unknown}\}$。

### 2. 前沿提取与目标候选

已知障碍按 `robot_radius` 膨胀，得到机器人中心不能进入的格集 $B$。
正常选点还将新鲜的 SCAN 膨胀障碍并入 $B$。设 $U$ 是当前搜索边界内的未知格集合，
$\mathcal N_4(q)$ 是格 $q$ 的上下左右四邻域，则前沿格集合为：

$$
F=\{u\in U\setminus B\mid
\exists q\in\mathcal N_4(u),\ M(q)=\mathrm{free}\}.
$$

前沿格用八邻域连接成簇 $C$；通常只保留 $|C|\geq m$ 的簇，
其中 $m=$ `min_frontier_cells`。簇中心仅用于**给簇内真实前沿格排序**，
不会把几何中心直接下发为目标。距离过近的候选会被跳过；近期失败、
刚到达但没有带来足够新观测的目标也会被暂时抑制。

### 3. 路径代价、评分与远距离分段

从机体当前位置 $s$ 所在的格运行八邻域 Dijkstra 搜索。设边 $e=(q,q')$ 的水平长度为
$\ell(e)\in\{r,\sqrt 2r\}$，未知格路径代价倍率为
$\lambda_u=$ `unknown_path_multiplier`，则前沿格 $f$ 的最小路径代价为：

$$
D(f)=\min_{\pi:s\leadsto f}\;
\sum_{(q,q')\in\pi}\ell(q,q')
\begin{cases}
\lambda_u,&M(q')=\mathrm{unknown},\\
1,&M(q')=\mathrm{free}.
\end{cases}
$$

路径不穿过膨胀障碍；斜向移动时，只要相邻的一个正交格被挡住，就不走该斜边。
未知格可以经过，但默认代价为已知空闲格的 $1.5$ 倍。对簇 $C$ 中按簇中心距离
排序后第一个通过路径、距离和抑制检查的前沿格 $f_C$，代码计算：

$$
J(C,f_C)=w_g\ln(1+|C|)-w_dD(f_C),
$$

其中 $w_g=$ `frontier_gain_weight`，$w_d=$ `path_cost_weight`。
各簇比较 $J$，选择得分最高的候选。若 $f_C$ 超过 `max_goal_distance`，
沿最短路径回溯，选择距离合适的**已观察空闲格** $g_C$ 作中途目标；
机器人到达后重新建图、重新选点。此时评分中的 $D(f_C)$ 仍是原前沿格的路径代价，
实际下发的是 $g_C$。

### 4. 无普通前沿与目标失败时的策略

若全图仍有可达未知格，却暂时没有普通前沿，节点沿已知空闲格寻找补充观测点。
它约每 $1\,\mathrm m$ 采样一个候选 $g$，在 `max_ray_range` 内统计从 $g$ 朝未知格
发射的、不被已知障碍截断的射线数 $V(g)$，最多计 $16$ 条，并使用启发式评分：

$$
J_{\mathrm{obs}}(g)=V(g)-0.25\,\|g-s\|_2.
$$

这里 $s$ 是机体当前水平位置。$V(g)$ 只是二维遮挡近似，不是完整的雷达视场或
信息熵计算。若还存在零散前沿但没有簇达到 $m$，持续 `no_frontier_hold` 后把
最小簇规模暂时降到一格再尝试。若有合格前沿簇，
但 `reachable_frontier_cells` 和 `candidate_goals` 都为零，节点可从
**本次真实行驶轨迹**中寻找已知空闲的回退点；回退未带来足够新观测时，会暂缓重复回退。

目标发出后，节点等待 SCAN 转发确认和到达回执。转发超时、目标持续落入新鲜的
SCAN 障碍图、长时间无有效前进或超过目标总时限时，先请求清除旧航点，再重新选点；
失败目标按 `failed_frontier_cooldown` 暂时屏蔽。已接近目标且持续停留时，
使用 `near_goal_dwell` 的近距离到达兜底。手动目标会暂停自动探索。

### 5. 可达区域覆盖与完成判定

全图模式先在配置矩形内统计原始未知格 $U$。完成验收另取**累计地图中的已知障碍**
膨胀集 $B_{\mathrm{map}}$，从机体附近的已知空闲格出发，在非障碍格上搜索连通区域
$R$；未知格在这一步仍视为潜在通路。暂时出现的 SCAN 障碍不参与验收，
以免一处瞬时障碍错误缩小完成范围。验收时仅在斜边两侧的正交格**都**被障碍
挡住时禁止斜向连通；这种判断比实际选点的路径检查更保守，优先避免漏算未知区域。

$$
U_{\mathrm{reach}}=U\cap R,\qquad
U_{\mathrm{excluded}}=U\setminus R,\qquad
\rho_{\mathrm{reach}}=1-\frac{|U_{\mathrm{reach}}|}{|R|}.
$$

`reachable_unknown_cells`、`excluded_unknown_cells`、`reachable_coverage_ratio`
分别对应 $|U_{\mathrm{reach}}|$、$|U_{\mathrm{excluded}}|$、$\rho_{\mathrm{reach}}$。
例如原始未知格还有 $114$ 个，但它们全在障碍内部或被已知障碍隔开时，
可达未知格为 $0$，这些格子不再阻止同层探索完成。只有全图边界有效、
起点可确定、点云和位姿新鲜、地图未满、没有活动目标、配置所需的 SCAN 障碍图新鲜，
且 $|U_{\mathrm{reach}}|=0$ 连续保持 `no_frontier_hold` 秒，才报告
`exploration_complete`。只要可达未知格再次出现，完成状态就在下一轮复核时撤销。
Gazebo 场景几何只供 `verify_flat_coverage.py` 独立验收，不参与在线地图和选点。

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

`exploration_complete` 仅在启用全图搜索、从机器人当前位置可连通的二维格均已有证据、
没有活动目标且点云/位姿及所需 SCAN 障碍图新鲜、地图未达容量上限，并持续复核
`no_frontier_hold` 秒后发布。连通计算使用累计点云形成的障碍并按机体半径膨胀；
未知格视为潜在通路，因此仅被已知障碍隔离或处于障碍膨胀范围内的未知格不阻止完成。
若起点附近找不到已观察的空闲格，则报告 `waiting_for_reachability_seed`，不宣布完成。
可达未知格刚清零时先报告 `waiting_for_coverage_confirmation`，持续满足复核时间后才报告完成。
全图逐格原始结果发布在 `/exploration/coverage_grid`
（`nav_msgs/OccupancyGrid`：`-1` 未知、`0` 已知空闲、`100` 已知障碍）。
状态同时给出 `total_cells`、`known_free_cells`、`known_occupied_cells`、
`unknown_cells`、`coverage_ratio` 和 `coverage_valid`；另有 `reachability_valid`、
`reachable_cells`、`reachable_unknown_cells`、`excluded_unknown_cells` 和
`reachable_coverage_ratio`。原始未知格满足
`unknown_cells = reachable_unknown_cells + excluded_unknown_cells`。
可达范围出现新的未知格或数据过期时，完成状态会在下一轮复核中撤销。
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
没有普通前沿但可达范围内还有未知格时，节点尝试在已观测、连通的空闲格上选择补充观测点；
发送时状态的 `detail=supplemental_observation`。若当前找不到可用观测位置，状态为
`unknown_regions_unobservable_or_unreachable`，`unresolved_unknown_cells` 给出本轮
没有可用目标时尚未覆盖的可达未知格数；它不等于永久不可达格数，也不会触发误报完成。
障碍物内部或已知障碍完全封闭区域的未知格单独计入 `excluded_unknown_cells`。
`map_capacity_reached` 表示体素上限已到，需增大容量或重启探索。

当前版本适用于同层、平坦场景的首次联调。未知区按未占据处理，不具备悬崖或
无地面支撑自动停车能力。普通入口仍按 `local_search_radius` 做局部搜索；
平层入口启用有边界的全图搜索。验收边界应与实际单层场地吻合；过大的边界会
包含无法观测的场地外格，导致任务始终保持未完成。

可用 Gazebo 平面场景做独立验收。仿真运行时，在另一个终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/likai/legbot_3D_Nav/install/setup.sh
ros2 run legbot_exploration verify_flat_coverage.py
```

脚本从 `ExplorationFlat.sdf` 的地面与静态碰撞体计算出生点可达网格，读取
`/exploration/coverage_grid` 并输出 `unobserved_reachable_cells`；Gazebo 几何仅用于
离线核验，不影响探索选点。默认出生点 `(-27,6)`、机体半径 `0.5 m`，可用
`--spawn-x`、`--spawn-y`、`--robot-radius` 调整。输出同时列出在线可达未知格和
在线排除未知格。若节点报告完成而真值可达网格仍有未知格，脚本返回非零退出码。

## 核心测试

```bash
PYTHONPATH=python python3 -m unittest discover -s test -p test_core.py
g++ -std=c++17 -Icpp/include cpp/src/exploration_core.cpp test/test_core.cpp -o /tmp/legbot_exploration_core_test
/tmp/legbot_exploration_core_test
PYTHONPATH=python python3 -m unittest discover -s test -p test_truth_coverage.py
```
