# legbot_exploration

第一次阅读源码可先看 [代码阅读指南](代码阅读指南.md)：它解释 ROS 话题、体素地图、前沿选点和状态机；关键语句与参数旁也已补上中文注释。

独立 ROS 2 Jazzy 功能包：探索节点使用 `explorer_cpp`。`python/` 仅保留独立的
Gazebo 真值覆盖率验收工具，通过 `ament_cmake_python` 安装。
旧 Python 探索器已移至项目根目录 `tmp/legacy_python_explorer_2026-10-29/`，
不参与当前构建或运行。

## 第一阶段：在线多层地形图（独立观察节点）

新增的 `online_layer_map_cpp` 与原有单层探索节点并行工作。它读取同一套适配器话题：
`/slam/global_cloud_registered`、`/slam/odometry_lidar` 和
`/slam/global_odometry_base`，在 `map` 坐标系中按时间匹配点云与雷达位姿，
沿射线累计三维占据证据。地图会随新点云更新；旧障碍被反复射线穿过后，
其证据可下降并从障碍转为空闲。

每个水平网格可以保存多个不同高度的候选表面。高度切片 `slice_height` 仅用来标记
候选所在的高度带，不会把它们合并成一层，也不把候选高度强制取整；同一水平位置的
上下两层可以同时存在。表面候选需要从上方观测到点云，避免将从下方看到的天花板
误当作可站立地面。候选再按邻近支撑点数、相邻高度差和上方占据证据分类。
`step_candidate` 仅表示有待后续验证的高度变化，不等于已经能够跨越。
未观测到顶面的露天位置归为 `unobserved`，不会仅因上方没有回波就判为有障碍；
它也没有被标记为“已验证净空”。完全没有点云证据的水平格不会凭空生成表面。
每帧保留至多 `max_points_per_cloud` 个点作为表面命中证据，同时只对其中至多
`max_rays_per_cloud` 个点执行射线追踪。这样既保留较密的地面观测，又限制每轮计算量；
两项默认分别为 12000 和 1200。表面颜色表示地形分类，不表示相机拍摄的真实颜色。

节点输出：

- `/exploration/layers/surfaces`：可在 RViz 用 PointCloud2 显示。绿色为有观测净空的平坦表面，蓝色为上方尚未充分观测的平坦表面，橙色为台阶候选，紫色为坡度或连接条件不合格，灰色为支撑不足，红色为净空被障碍占用。点云还包含 `slice_index`、`surface_kind`、`clearance_kind`、`support_columns`、`ceiling_z`、`hit_observations` 和 `last_observed` 字段，供后续跨层模块读取。
- `/exploration/layers/ceilings`：已观测到的上方障碍点，RViz 中可用 PointCloud2 显示。
- `/exploration/layers/status`：JSON 统计，包括体素、水平格、候选表面数量、点云同步丢弃数、地图容量状态和每轮更新耗时。
  `last_raw_cloud_points`、`last_sampled_points`、`last_rays_requested` 和
  `integration_ms` 可用于检查点云抽样与射线追踪是否正常。
  在线地图体素上限为 800 万；`map_full=true` 表示上限已满，后续新区域无法继续入图。
  修改此上限后需重启地图节点，已运行节点不会自动读取新值。

仅测试在线地图时，先关闭已单独运行的 Gazebo、定位适配器和旧地图节点，
然后用一条命令启动 Gazebo、GUI、Gazebo 真值适配器、在线多层地图和 RViz：

```bash
cd /home/likai/legbot_3D_Nav
source /opt/ros/jazzy/setup.bash
source install/setup.sh
ros2 launch legbot_exploration multi_layer_map.launch.py
```

默认开启 Gazebo GUI 和 RViz。无显示器时可加
`gazebo_gui:=false start_rviz:=false`；这不会改变点云建图与话题发布。
默认世界为 `Building.sdf`。需要平面场景时，可给同一命令加上
`world:=/home/likai/legbot_3D_Nav/install/legbot_bringup/share/legbot_bringup/worlds/ExplorationFlat.sdf`。
RViz 使用本包的 `rviz/multi_layer_map.rviz`，启动后自动订阅
`/exploration/layers/surfaces` 和 `/exploration/layers/ceilings`，并显示 GO2 模型；
固定坐标系为 `map`。表面按小方块显示，原点处的参考网格已隐藏，避免机器狗
出生在远离原点的位置时将网格误认为地形边界。
查看状态可运行 `ros2 topic echo /exploration/layers/status`。
此入口不启动自动探索、SCAN 或 GO2 高层行走控制，因此机器狗不会自己去扫描整张地图。
若要让机器狗边探索边累计多层图，先启动已有完整探索入口，再另开终端运行
`ros2 launch legbot_exploration multi_layer_map.launch.py start_simulation:=false start_rviz:=false`。
这样只添加地图节点，不会重复启动 Gazebo、定位适配器或第二个 RViz。

也可以只用一条命令启动 Gazebo、定位、GO2 控制、SCAN、自主探索、在线多层地图
和多层地图 RViz（只启动一份 Gazebo 与 RViz）：

```bash
ros2 launch legbot_exploration multi_layer_exploration_demo.launch.py
```

默认使用 Building 场景和 Gazebo 真值定位；可用
`localization_source:=fastlio|fastlivo` 选择估计定位，或用 `world:=...` 指定场景。
`control_start_delay:=12.0` 控制 GO2 高层控制的启动延时，
`show_layer_rviz:=false` 可关闭默认打开的地图 RViz。此入口会自动发送探索目标，
与上面的纯地图观察命令不同。启动前先关闭之前的仿真与探索进程，避免话题重复发布。
组合入口在 Building 场景为探索选点额外启用 SCAN 障碍图就绪检查、0.50 m
目标避障半径和 4 m 单次目标距离；同时将 SCAN 局部障碍膨胀半径设为
0.35 m，减少贴着圆柱规划轨迹的情况；可用 `scan_robot_radius:=数值` 调整。
探索节点读取的是 SCAN 已膨胀的障碍图，只再补足到自身 0.50 m 安全半径的差额，
避免把同一障碍重复膨胀；调整 `scan_robot_radius` 时会同步更新这一差额。
修改只影响新组合入口，
`exploration_demo.launch.py` 的单独启动参数保持原有默认值。组合入口的
`show_layer_rviz` 控制唯一的 RViz 窗口，`start_rviz` 不用于控制这个窗口。
启动后可用 `ros2 topic info /exploration/status -v` 确认探索状态发布者存在，
再用 `ros2 topic echo /exploration/status std_msgs/msg/String --once --qos-durability transient_local --full-length` 读取完整状态。

起步时如果机器人脚下仍有点云盲区，探索器进入“起步建图”状态。GO2 站稳且
里程计、点云和 SCAN 障碍图就绪后，机器人以 0.35 rad/s 原地转一圈；完成后
只发零角速度，不再重复发送模式 2，以免已站立的机器狗切换成趴下。等待运动
停止后，从附近 0.65–2.0 m 的已观测空闲格里选一个机体膨胀范围也空闲的
目标，交给现有 SCAN 航点接口。已知障碍不能穿过；起点至目标之间仍有未知
部分时，由 SCAN 检查并规划实际短距离运动。目标点本身还须通过新鲜层析图
的支撑和净空检查。若没有安全目标、转向超时或位姿失效，节点停车并在
`/exploration/status` 报告 `bootstrap_failed`。参数 `startup_scan_enabled`、
`startup_yaw_rate`、`startup_turn_timeout` 和起步目标距离范围见
`config/exploration.yaml`。

在线多层地图节点本身只建立、更新和显示地图，不发布导航目标；一键组合入口中的
导航目标仍由原有单层探索节点产生，SCAN 负责执行。原有单层探索入口不启动此地图节点，
保持原来的行走策略。

## 第二阶段预览：表面图与跨层候选 A*

在线多层地图节点现在还会根据已观测表面建立一张只读的局部表面图。图的每个节点是
一个具体 `(x, y, 实测高度)` 表面；同一水平格的上下表面保持为不同节点。只有相邻
水平格均有足够支撑、没有已知净空障碍，且高度差不超过坡度或单级台阶阈值时才连边。
搜索以机体当前位置附近、与 `机体 z - graph_nominal_body_height` 相符的表面为起点，
用 A* 寻找图上可达且高于起点的橙色台阶候选。搜索半径、节点数和高度阈值在
`config/multi_layer_map.yaml` 中配置；地图每轮更新后都会重建预览，旧路径会被清除。

RViz 中浅灰点是抽样显示的图节点，浅绿细线是抽样显示的相邻连接，黄色球是上升台阶
候选，紫红粗线是 A* 找到的候选路线。路线的高度是地面表面高度，尚未换算成机体目标
高度。`/exploration/layers/graph_preview` 发布这些 Marker；
`/exploration/layers/candidate_path` 发布同一路径的 `nav_msgs/Path`，仅供检查，
不会进入 `/scan/goal` 或 `/scan/waypoint_input`。

`/exploration/layers/status` 新增 `graph_state`、`graph_nodes`、`graph_edges`、
`graph_rising_candidates`、`graph_path_points` 等字段。`graph_nodes_truncated=true`
表示局部候选表面超出本轮搜索节点上限，搜索结果只覆盖保留的节点。`preview_path_observed` 表示
路径上表面的净空已有观测证据；`preview_path_clearance_unobserved` 表示路径上还有
未确认净空。`rising_candidates_unreachable_in_window` 只表示当前局部搜索窗口内
候选不可达，不能据此判定整张地图没有楼梯。表面图只检查局部高度、支撑和净空，
尚未完成机体扫掠体积、足端落点及动态执行验证，因此路径不能直接用于跨层控制。

## 输入和输出

- 输入：统一适配器输出的 `/slam/global_cloud_registered`、
  `/slam/odometry_lidar`、`/slam/global_odometry_base`，以及 `/go2/demo_ready`。
  适配器可选择 Gazebo 真值、FAST-LIO2 或 FAST-LIVO2。若 SCAN 正在运行，还会读取
  `/local_planner/grid_map/occupancy_inflate` 过滤被规划器判为障碍的目标。
- 输出：`/exploration/frontiers` (MarkerArray)、`/exploration/status` (JSON String)，
  `/exploration/planned_path` (Path) 以及由机体里程计累计的
  `/exploration/trajectory` (Path)。轨迹从探索节点启动时
  开始记录，每移动约 0.1 m 增加一点，RViz 中显示为绿色线。平层 RViz 固定
  `map` 视角概览整块场地，三种定位来源使用同一个视图。
- 自动导航时，目标发到 `/scan/waypoint_input`，由既有
  `scan_waypoint_mission` 以 `direct` 模式转发到 `/scan/goal`。节点根据转发目标的
  时间戳、`/scan/goal_stop_ack` 和 `/scan/mission_complete` 判断本次路点完成；
  探索节点随后复查并发送所选路径的下一路点。
  若机器人已在 `goal_reach_xy` 内停留 `near_goal_dwell` 秒而 SCAN 未回执，
  探索节点先请求清除旧路点，成功后以 `goal_reached`、
  `detail=proximity_fallback` 记录到达，避免反复记为 `no_progress`。
- `/exploration/set_paused` 是 `std_srvs/SetBool`：`true` 暂停且清除自动目标，
  `false` 继续。手工在路点输入话题发目标时也会自动暂停探索。

## 探索方案、公式与策略

本包按“配准点云与位姿 → 累计三维证据 → 同层二维投影 → 前沿聚类与选点 →
同层层析图或二维占据图规划 → SCAN 逐段执行 → 根据新观测重新选点”的闭环运行。
探索节点包含双路线选择和逐段执行；定位来源由上游适配器选择，
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
全图模式先在配置矩形内选点；若没有可用目标，则从完整累计地图中寻找矩形外的
候选前沿。矩形外候选必须属于机器人所在的已知空闲连通区域的原始前沿。
完整地图的选点搜索只扩展到已观察网格的外缘一格；动态包围盒超过二百万格时
不执行该轮路径搜索，并保留“前沿未解决”状态，不宣布完成。

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

路径的中间格只使用已观察空闲格，未知格仅能作为前沿终点；斜向移动时，
两个相邻正交格也必须是已观察空闲格。前沿终点的代价按默认倍率 $1.5$ 计入。
对簇 $C$ 中按簇中心距离
排序后第一个通过路径、距离和抑制检查的前沿格 $f_C$，代码计算：

$$
J(C,f_C)=w_g\ln(1+|C|)-w_dD(f_C),
$$

其中 $w_g=$ `frontier_gain_weight`，$w_d=$ `path_cost_weight`。
各簇比较 $J$，选择得分最高的候选。若 $f_C$ 超过 `max_goal_distance`，
沿最短路径回溯，选择距离合适的**已观察空闲格** $g_C$ 作候选观察目标；
此时评分中的 $D(f_C)$ 仍是原前沿格的路径代价。C++ 执行器再为候选目标生成
一条经过已观察空闲格的路线：优先尝试同层层析表面图 A*；若没有可用表面路径，
则使用二维占据图 A*。两种路径只用于选择参考目标点，不替 SCAN 决定具体走法。
按路径长度每约 `route_segment_length`（默认 3 m）选一个已知空闲且膨胀安全的目标点，
整条路线最多选 3 个，通过 `/exploration/planned_path` 显示，并依次发送到
`/scan/waypoint_input`。地图更新后只复查尚未发送的目标点及当前目标点；
目标点失去安全余量时清除该目标并重新选点。两类规划共用同一个执行接口。
两种路线共用下发前的机体占用圆盘检查：以 `robot_radius + waypoint_safety_margin`
为半径，圆盘覆盖的每个二维格都必须已有空闲观测；未知格和已知障碍都拒绝该路点。
还会复核新鲜 SCAN 膨胀障碍图；层析图新鲜时，检查路点的地面支撑和已知净空障碍。
地图更新后，活动路点和下一个路点都会重新检查。机器人当前所在格不作为
目标点安全门槛；路点之间的实际运动路线由 SCAN 根据实时障碍规划。
对于未知前沿，执行终点还须能从已知空闲格直视该前沿，并比基本机体半径多留
`0.15 m` 的已知障碍余量。

### 4. 无普通前沿与目标失败时的策略

若完整地图仍有原始前沿，但受瞬时 SCAN 障碍等影响暂时没有可选前沿，
且矩形覆盖统计仍有可达未知格，节点沿已知空闲格寻找补充观测点。
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

### 5. 全局原始前沿与完成判定

配置矩形内的覆盖率网格继续用于显示和独立仿真验收，不再单独决定探索完成。
完成检查使用未按矩形裁剪的完整累计地图。只将累计点云中的已知障碍按
`robot_radius` 膨胀为 $B_{\mathrm{map}}$；瞬时 SCAN 障碍不参与完成检查。
从机器人附近的已知空闲格出发，仅沿四邻接、已知空闲且不在 $B_{\mathrm{map}}$
内的格子搜索，得到当前连通区域 $R_{\mathrm{free}}$。其外缘的未知格就是
**全局原始前沿**：

$$
F_{\mathrm{raw}}=\{u\notin M\cup B_{\mathrm{map}}\mid
\exists q\in R_{\mathrm{free}},\ u\in\mathcal N_4(q)\}.
$$

这里 $M$ 表示累计二维投影中已经有观测证据的格子集合。只需检查有限的
已知空闲格及其四个邻格，不需要预先枚举无限大的未知区域。原始前沿不受
最小簇规模、路径距离、目标冷却及瞬时 SCAN 障碍筛选影响。

`global_frontier_valid` 表示机器人附近找到了已知空闲起点；
`global_reachable_free_cells` 是 $|R_{\mathrm{free}}|$；
`global_frontier_cells` 是 $|F_{\mathrm{raw}}|$。
只有起点有效、原始前沿数持续为零、点云与位姿新鲜、地图未满、没有活动目标，
且配置要求的 SCAN 障碍图新鲜，持续 `no_frontier_hold` 秒后，才报告
`exploration_complete`。出现新前沿时立即取消完成计时。

作为辅助诊断，原有矩形统计仍计算 $U$、累计障碍膨胀后的潜在连通区域 $R$
及其未知格数量；此处未知格仍可作为潜在通路：

$$
U_{\mathrm{reach}}=U\cap R,\qquad
U_{\mathrm{excluded}}=U\setminus R,\qquad
\rho_{\mathrm{reach}}=1-\frac{|U_{\mathrm{reach}}|}{|R|}.
$$

`reachable_unknown_cells`、`excluded_unknown_cells`、`reachable_coverage_ratio`
分别对应 $|U_{\mathrm{reach}}|$、$|U_{\mathrm{excluded}}|$、$\rho_{\mathrm{reach}}$。
这些矩形统计量用于观察覆盖进度，不能代替 $F_{\mathrm{raw}}$ 的完成检查。
例如，`global_frontier_valid=true`、`global_reachable_free_cells=13315`、
`global_frontier_cells=0`，同时矩形内 `unknown_cells=115`、
`excluded_unknown_cells=115`，表示机器人当前连通的已知空闲区域外缘没有
原始前沿；矩形内仍有 115 个未观察格，但它们不在矩形统计所用的潜在连通区域内。
这两个连通搜索不同：完成检查的 $R_{\mathrm{free}}$ 只经过已知空闲格，
矩形覆盖统计的 $R$ 允许未知格作为潜在通路。计数会随新点云和位姿继续复核。
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
`control_start_delay:=20.0` 调整。需要观察模式时，使用下方
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

只想观察选点时，将 `observation_only:=false` 改为 `true`。

探索节点读取 `config/exploration.yaml`。地图逐帧用已配准点云及同时间戳的
雷达、机体位姿做射线更新：观察过的空闲体素、障碍体素和未观察体素分开保存。
体素的 Z 以**观测时的机体高度**为原点，故同层行走时定位的整体高度漂移不会
把早期观测从当前投影中排除。路径中间格限定为已观察空闲格；前沿未知格
只用于选取观察方向，执行目标落在其附近的已观察空闲格。候选前沿仍是 Frontier 簇里的实际未知格，
而非可能落在障碍内的几何质心。已观察的障碍和新近收到的 SCAN 膨胀障碍都会
参与筛选；活动目标若持续落入 SCAN 障碍，会提前撤销并暂时抑制。

高度归一化面向**同层探索**。跨层时相同 XY 的不同楼层可能重叠，需要独立的
楼层子图及楼层切换逻辑，不能直接把此模式用于跨层完成判定。若 SCAN 占据话题
不可用，节点仍可依靠自身地图选点，但会失去与 SCAN 障碍的一致性检查。

`exploration_complete` 仅在全图模式下，完整累计地图的机器人连通空闲区域
连续 `no_frontier_hold` 秒没有原始前沿，并满足数据新鲜、地图容量和活动目标检查时发布。
矩形覆盖率为 $100\%$ 但矩形外仍有原始前沿时，继续向矩形外选点。
若起点附近找不到已观察的空闲格，则报告 `waiting_for_reachability_seed`，不宣布完成。
原始前沿刚清零时先报告 `waiting_for_coverage_confirmation`，持续满足复核时间后才报告完成。
配置矩形内的逐格覆盖结果发布在 `/exploration/coverage_grid`
（`nav_msgs/OccupancyGrid`：`-1` 未知、`0` 已知空闲、`100` 已知障碍）。
它不是完整累计地图的逐格话题；完整累计地图的连通域验收结果应查看
`/exploration/status` 中的 `global_frontier_valid`、
`global_reachable_free_cells` 和 `global_frontier_cells`。
状态同时给出 `total_cells`、`known_free_cells`、`known_occupied_cells`、
`unknown_cells`、`coverage_ratio` 和 `coverage_valid`；另有 `reachability_valid`、
`reachable_cells`、`reachable_unknown_cells`、`excluded_unknown_cells` 和
`reachable_coverage_ratio`。原始未知格满足
`unknown_cells = reachable_unknown_cells + excluded_unknown_cells`。
完整累计地图出现新原始前沿或数据过期时，完成状态会在下一轮复核中撤销。
`current_region_complete` 只用于局部搜索模式，说明当前高度和搜索半径内持续没有 Frontier。
`frontiers_below_cluster_threshold` 表示仍有零散前沿，但尚未达到簇规模门槛；
单层全图模式持续这种状态 8 秒后，会按单格前沿再次尝试选点，目标仍须通过路径、距离和障碍筛选。
`frontiers_unreachable_or_suppressed` 表示仍有前沿簇，但暂时无法选用。
`global_frontiers_unresolved` 表示全局原始前沿仍存在，但本轮选点经过
SCAN 障碍等筛选后没有可选前沿格；此状态不会宣布完成。
状态中的 `frontier_cells` 和 `frontier_clusters` 分别给出本轮普通选点搜索中的
候选前沿格数和通过规模门槛的簇数；它们与完成验收用的
`global_frontier_cells` 不是同一个计数。
`reachable_frontier_cells`、`candidate_goals`、`suppressed_goals`、`open_start_neighbors`
用于区分地图断连、目标冷却与起点周围已观察空闲格的数量。只有存在合格前沿簇、
却没有可达前沿格和候选目标时，全图模式才会从本次实际行驶轨迹里选择约 2–4 m
的已观察空闲回退点；状态的 `detail=recovery_backtrack` 表示正在执行这一步。
回退后若新增已观测 XY 格少于 `recovery_min_new_cells`，暂不重复回退；到达目标后
若观测增长不足，也暂不重复选择同一目标。地图新增足够格子后解除这两项抑制。
因此仍有前沿但都不可用时，节点会报告未解决的前沿状态，不会宣布探索完成。
没有普通前沿但可达范围内还有未知格时，节点尝试在已观测、连通的空闲格上选择补充观测点；
发送时状态的 `detail=supplemental_observation`。若当前找不到可用观测位置，状态为
`unknown_regions_unobservable_or_unreachable`，`unresolved_unknown_cells` 给出本轮
没有可用目标时尚未覆盖的可达未知格数；它不等于永久不可达格数，也不会触发误报完成。
`excluded_unknown_cells` 是配置矩形内、不属于当前潜在连通区域的未知格数量；
其中可能包含障碍膨胀覆盖或已知障碍隔开的格子，不能仅凭这一计数逐格判定
未知格的物理成因。
`map_capacity_reached` 表示体素上限已到，需增大容量或重启探索。
`route_source` 表示当前选用了 `layer` 或 `occupancy` 参考路线，`route_stages` 和
`route_stage_index` 表示待发送目标点总数和当前序号。二维图即使在机器人当前
位置附近尚未形成连通参考路径，也只按已知空闲及膨胀余量寻找安全观察目标；
是否能到达该目标由 SCAN 的实时规划决定。

当前版本适用于同层、平坦场景的首次联调。未知区按未占据处理，不具备悬崖或
无地面支撑自动停车能力。默认 `global_search: true`，先在配置矩形内选点，
必要时扩展到完整累计地图；如需局部搜索，可在参数文件中显式设置 `false`。
矩形覆盖网格用于
显示和仿真验收；它的边界不再决定探索完成。

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
g++ -std=c++17 -Icpp/include cpp/src/exploration_core.cpp test/test_core.cpp -o /tmp/legbot_exploration_core_test
/tmp/legbot_exploration_core_test
PYTHONPATH=python python3 -m unittest discover -s test -p test_truth_coverage.py
```
