# legbot_exploration

当前 PCT 选点使用[三维局部窗口优先、全局后备](docs/pct_local_window.md)，
窗口只限制目标选择，原生 PCT 仍在完整累计图上规划。

第一次阅读源码可先看 [代码阅读指南](代码阅读指南.md)：它解释 ROS 话题、体素地图、前沿选点和状态机；关键语句与参数旁也已补上中文注释。

独立 ROS 2 Jazzy 功能包：`combined_exploration_cpp` 是唯一编译的生产可执行文件，
通过 `--exploration-mode=flat|map|combined` 选择单层探索、仅建图或组合模式。
旧命令 `explorer_cpp` 和 `online_layer_map_cpp` 是指向同一文件的兼容链接。
`python/` 仅保留独立的
Gazebo 真值覆盖率验收工具，通过 `ament_cmake_python` 安装。
旧 Python 探索器已移至项目根目录 `tmp/legacy_python_explorer_2026-10-29/`，
不参与当前构建或运行。

PCT 定向补扫、短距离换位及 SCAN 委托共用下文的执行接口。

## 探索控制与路径规划的第一步拆分

探索节点是唯一向 `/scan/waypoint_request` 发送探索路点的节点。
选定前沿目标后，它给本次请求分配 `goal_id`、`attempt_id`、二维累计地图版本和
层析图版本。`route_orchestrator.cpp` 对同一次目标请求分别计算二维与层析候选；
组合入口的层析规划在读锁内直接使用在线地图，路径选择只接受编号和版本均匹配的结果。
普通目标按规划器配置选择路线。PCT 平层探索按楼层记录已确认的楼梯入口坐标和上行方向，
可在 `/exploration/status` 的 `pct_floor.saved_stair_entrances` 查询。
本层首次与复查队列处理完成后开始查找上行路线。已有踏面线索时，可先从同层安全站位定向补扫。
地图产生新版本后，已下发的路点继续等待 SCAN 回执，再依次推进。

`/exploration/planning_goal` 和 `/exploration/planning_results` 发布带编号、
版本及规划原因的 JSON 诊断；`/exploration/plans/occupancy`、
`/exploration/plans/layer`、`/exploration/plans/recovery_layer` 发布各候选路径，
`/exploration/planned_path` 是最终选中、将分段交给 SCAN 的路点。
当前规划模块通过包内 C++ 接口交互，这些话题用于观察和核对，
尚未作为独立进程之间的请求与结果传输接口。
组合入口中的层析规划直接读取同进程在线地图的表面和占据索引。单层模式
只做二维规划，不订阅层析规划点云。层析规划不再调用二维占据格或 SCAN
膨胀点对其节点、路段和执行路点做交叉检查。层析规划器自行验证支撑、已观测
净空、障碍与机体扫掠空间，并输出原始 A* 参考路径及最终执行路点。路径处理
模块对层析路点按原顺序和坐标下发，不简化、不插点，也不跳过附近路点；仅对
普通二维 A* 路线进行安全约束下的简化。当前层析路线仍限于同层，跨层执行
尚未接入。

SCAN 在直达目标的全局参考路径生成失败，或当前目标的局部规划连续失败达到
`fsm.max_replan_fail_count` 时，向 `/scan/goal_plan_failed` 发布一次终态回执。
消息是 `std_msgs/String`，内容为 `转发目标时间戳纳秒|原因`；原因包括
`no_body_pose`、`global_reference_failed`、`global_target_occupied` 和
`local_replan_exhausted`。单次局部重规划失败不会结束目标。
路径处理模块只接收与当前路点 `/scan/goal` 时间戳一致的回执，忽略旧目标消息；
成功仍需匹配的停车应答和任务完成信号。它把成功或失败统一发布到
`/exploration/execution_feedback`，附带 `goal_id`、`attempt_id`、
`stage_index`、`scan_goal_stamp`、规划来源及原因。收到终态失败后，探索控制
才结束当前路线，并按当前前沿簇或跨层任务处理失败。
`/exploration/status` 同时记录 `scan_failure_count` 和最近一次失败原因。
航点协调器保持原样。

## 执行职责与脱困动作

exploration 不发布 `/go2/control_mode` 或 `/cmd_vel`，也不再发布旧
`/exploration/bootstrap_turn_cmd`。探索控制只决定目标与脱困动作；路径处理器按当前
时间戳接收结果，并发布统一的 `/exploration/execution_feedback`。

- 原地转向：`/scan/recovery_turn_request`，String 内容 `时间戳纳秒|相对角度弧度`。
  SCAN 执行适配器累计实测转角、限制位置漂移并判定超时；成功发布
  `/scan/recovery_turn_result` 的 `时间戳|succeeded`。
- 保持朝向直退沿用 `/scan/recovery_reverse_request` 与 `/scan/recovery_reverse_result`。
- 两类动作失败均通过 `/scan/goal_plan_failed` 返回匹配时间戳的原因。
  暂停探索或手动接管时可发送 `/scan/recovery_action_cancel`（String，当前时间戳）；
  地图更新不会触发动作撤销，exploration 不自行监测执行无进展。

执行侧闭环：适配器发布 `/scan/recovery_action_state` 的 `时间戳|accepted`，
`go2_demo_sequencer` 完成原有模式 3 激活脉冲后，发布同时间戳的
`/scan/recovery_action_enabled` 心跳。适配器收到后才运动；未获许可 4 秒、许可
中断超过 0.5 秒均停车并返回失败。动作结束发布 `时间戳|finished`，协调器结算当前执行。
探索通过可靠、保留最新值的 `/exploration/task_active`（Bool）发布任务生命周期：
建图、规划、脱困与路点间等待均为 true，确认完成、暂停或仅观测模式为 false。
探索持续时，执行侧保持已激活的 RL，不在每个路点后回固定站立。
任务允许结束且当前执行结算后，协调器至少等待 `goal_settle_seconds`（默认 1.5 秒），
再依据新鲜的零速度指令和连续机体位姿确认停稳，才请求模式 2；活动信号本身不激活步态。
暂停不伪造当前目标已结束的回执，仍须等待 SCAN 结算。
阈值见 `legbot_bringup/config/go2_execution.yaml`，执行侧检查结果见 `/go2/execution_state`。
旧动作、旧取消和旧结果不能替代当前动作的身份。

执行适配器参数 `recovery_turn_speed` 默认
0.35 rad/s、`recovery_turn_timeout` 默认 8 s；直退速度仍由
`recovery_reverse_speed` 控制。这些是 SCAN 适配器参数，不属于探索选点配置。

隔离执行测试：`test/test_execution_actions_ros.py` 运行真实适配器和协调器，模拟里程计，
验证行走激活门槛、转向跨越 ±π、直退、过期回执、取消、超时和许可中断；不会连接真机。

## 在线 PCT 层析图

组合入口与仅建图模式均使用唯一的在线 PCT 图。节点直接累计
`/slam/global_cloud_registered` 的全部有限 XYZ（必须为 map 坐标系），
仅用真实命中点更新 G/C；可选下视雷达的注册点云也融合到同一张图。
不补面、不拟合或外推未知地面，不读取二维占据图。
机体里程计仅用于选点、规划和上升预览。固定世界原点、0.10米水平格、G/C、
净空、梯度和机体/未知膨胀均见 [在线PCT说明](docs/online_pct.md)。
在线 PCT 使用固定大小分块、同列共享的有序表面记录和依赖驱动的局部代价更新。
本轮仅支持静态模式：未成熟表面按点数加权更新平均高度，默认3个真实点即成熟。
成熟后忽略容差内重复点；容差外的新高度仍接受为新证据，不会被旧格的冻结逻辑丢掉。
`pct_surface_noise_tolerance` 当前 YAML 试验值为0.05米，`pct_maturity_min_points` 默认3；
同一物理表面的多个切片共享 G/C 来源，不会因地面跨切片波动生成极小净空。
不再维护三维均值、散布矩阵、协方差或动态时间窗口。详见[在线 PCT](docs/online_pct.md)。

组合节点在仿真开始时即启动，控制与 SCAN 按原顺序延时启动，探索等执行就绪才发运动目标。
缺测区域保持未知，不补面；下视雷达在多层 demo 默认开启，见[下视雷达测试](docs/ground_lidar_test.md)。

节点输出：

- `/exploration/layers/surfaces`：新PCT表面的抽稀显示，绿色安全实测、灰色规划受限、蓝色证据不足、红色实体/陡坡。
- `/exploration/layers/planning_surfaces`：新PCT全部有限G表面，含cost、source、flags、ceiling_z、clearance、slice_index字段。均为发布输出，内部规划仍直接读共享图。
- `/exploration/layers/measured_returns`：累计表面记录的真实命中高度上界，不表示经过射线清除的占据概率；旧planning_occupied话题已删除。
- `/exploration/layers/ceilings`：新PCT中有实际上方命中的C。
- `/exploration/layers/status`：后端、版本、接受/规划丢帧、容量、PCT统计、预览状态、构图耗时；另含每路点云的接收和融合计数。

`publish_map_topics` 默认开启，关闭后仍更新PCT与状态，不产生地图显示输出。
显示抽稀仅影响surfaces话题；完整表面导出不抽稀，不改变内部规划。
规划持共享图读锁，写线程在规划期间丢弃帧，不复制规划全图。

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
Building 场景的二维覆盖栅格显示范围为 `x∈[-31.5,31.5]`、
`y∈[-15.5,15.5]` 米，包含东侧建筑；纯平面入口仍使用它自己的相对边界。
`control_start_delay:=12.0` 控制 GO2 高层控制的启动延时，
`show_layer_rviz:=false` 可关闭默认打开的地图 RViz。此入口会自动发送探索目标，
与上面的纯地图观察命令不同。启动前先关闭之前的仿真与探索进程，避免话题重复发布。
探索、在线层析图及两路雷达近距过滤统一读取 `legbot_bringup/config/robot_geometry.yaml`。
规划机体半径 `robot_radius` 为 0.35 m，安全余量另为 0.05 m；
两路雷达各自原点的三维近距过滤由独立参数 `lidar_min_range: 0.70` 控制。
省略它时回退使用机体半径；启动安全开关不会覆盖机体半径。
两个探索 demo 入口的 SCAN 局部障碍膨胀半径默认也为 0.35 m，
可用 `scan_robot_radius:=数值` 单独调整这一执行侧参数。
二维分支读取 SCAN 已膨胀障碍图时，只补足与自身机体半径的差额；
纯 PCT 分支独立检查层析图，不依赖 SCAN 二维障碍图，也不限制目标距离。
组合入口的
`show_layer_rviz` 控制唯一的 RViz 窗口，`start_rviz` 不用于控制这个窗口。
启动后可用 `ros2 topic info /exploration/status -v` 确认探索状态发布者存在，
再用 `ros2 topic echo /exploration/status std_msgs/msg/String --once --qos-durability transient_local --full-length` 读取完整状态。

起步时只等待里程计、点云和 GO2 控制就绪，不自动转圈。若脚下盲区使正常规划失败，
或同一位置附近反复选点、规划失败，探索控制进入脱困状态：先尝试附近已观测且机体圆盘
安全的短目标；随后依次尝试 0.22～0.60 米的保持朝向直退，必要时分步原地转向，
最后尝试本次行驶轨迹上的安全回退位置。每一步都有次数和超时限制。脱困目标
可以从脚下盲区或膨胀区起步，但新扫过的未知格与障碍不可进入，已有障碍的距离
必须增加。直退一次后等待新地图，仍在膨胀区则继续脱困。直达路线标记为
`recovery_unverified`，由 SCAN 决定实际运动。所有方法均失败时
报告 `recovery_blocked`，保留未解决前沿，停车等待地图更新或人工处理。

`/exploration/diagnostics/goal_event` 在目标下发和 SCAN 失败时记录目标、起始与
当前位姿、地图版本、路线来源及到原始/膨胀障碍的距离。失败时另保留
`/exploration/diagnostics/last_reference_path`、`last_actual_path` 和
`last_scan_bspline`，分别对应探索参考路点、机体实走里程计与 SCAN 最新 B 样条。

在线多层地图节点本身只建立、更新和显示地图，不发布导航目标；一键组合入口中的
导航目标仍由原有单层探索节点产生，SCAN 负责执行。原有单层探索入口不启动此地图节点，
保持原来的行走策略。

## 上升预览：直接使用新 PCT 与原生 A*

不再维护额外的旧表面图或兼容A*。节点在新PCT安全表面中按实际高度寻找
上升候选，调用与正常规划相同的原生PCT A*和路段检查，输出地面参考高度路径。
当前位置不满足普通PCT准入时明确报unsafe_start，不移到附近节点伪造起点。
候选搜索半径默认7米，仅约束预览候选采样，不是正常规划范围限制。
`graph_max_preview_candidates` 默认8，限定每轮最多尝试的目标数；预览是可选计算。

`/exploration/layers/graph_preview`显示安全节点、橙色上升目标和紫红色路线，
`/exploration/layers/candidate_path`输出同一路径，二者从不发送运动命令。
状态包含graph_state和graph_path_points。找不到路线只表示本轮预览没有结果，
不能证明全图没有楼梯，也不能替代后续跨层任务控制和运动能力验证。

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
- 自动导航时，目标发到 `/scan/waypoint_request`，由既有
  `scan_waypoint_mission` 以 `direct` 模式转发到 `/scan/goal`。节点根据转发目标的
  目标标识、时间戳及执行器 `/scan/goal_stop_ack` 判断本次扩展路点完成；
  旧 PoseStamped 输入保留 `/scan/mission_complete` 兼容流程，近路点回执先到时会暂存核对；
  完成后发送所选路径的下一路点。目标发出后，exploration 不按地图变化、
  距离或计时器撤销 SCAN 目标；未收到完成回执时保持等待。
- `/exploration/set_paused` 是 `std_srvs/SetBool`：`true` 暂停后续自动选点，
  不撤销已下发目标；`false` 继续。手工在路点输入话题发目标时会接管当前任务并暂停自动探索。

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

### 3. 路径代价与前沿评分

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
排序后第一个通过路径和抑制检查的前沿格 $f_C$，代码计算：

$$
J(C,f_C)=w_g\ln(1+|C|)-w_dD(f_C),
$$

其中 $w_g=$ `frontier_gain_weight`，$w_d=$ `path_cost_weight`。
各簇比较 $J$，选择得分最高的真实前沿目标，不按目标距离替换成中途格。
探索控制先在前沿附近
选出同一个已观测、膨胀安全的观察位置。二维占据 A* 和
层析PCT A* 分别在各自地图上规划到它；层析路线合格时优先选择，失败时
使用二维路线。二维 A* 的节点和边检查完整机体圆盘及 SCAN 膨胀障碍，未知格不能
作为普通路径的中间格。层析正常与脱困规划均使用唯一在线PCT，保留净空、梯度和机体/未知膨胀检查，
层析规划器自行输出执行路点；
路径处理模块只简化二维路线，且直线替代仍需满足走廊安全与观测代价约束。
最终路点显示在
`/exploration/planned_path`，按次序通过 `/scan/waypoint_request` 交给 SCAN。
正常规划不会生成未验证的直达线。执行期间的地图或机体位置变化不会使探索器
清除已选路线；所有规划出的目标点须在已观察空闲区，
机体圆盘覆盖的未知格或障碍使其不合格。
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
信息熵计算；补充观察点函数目前保留但不调用。若还存在零散前沿但没有簇达到 $m$，
持续 `no_frontier_hold` 后把最小簇规模暂时降到一格再尝试。若两路规划均明确报告
起点不安全，控制模块累计失败并进入脱困；其他规划失败保留未解决前沿，继续尝试
其他候选。控制模块依次选择附近目标、直退、转向及历史点，脱困规划只验证路线。

目标发出后，节点只根据 SCAN 转发目标、停车回执和任务完成信号推进路线。
规划阶段无法形成路径的候选目标按 `failed_frontier_cooldown` 暂时屏蔽。
SCAN 未完成时，exploration 不重新选点或清除航点；手动目标可以接管自动探索。

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

实现时，累计体素证据同步维护二维列状态；某格在“空闲、障碍、未知”之间变化时，
只重算该格及四邻格的前沿资格。索引保存当前所有原始前沿，新增观测会移除旧前沿、
生成新前沿；障碍证据被后续空闲射线抵消时也会更新。每轮仍从机器人所在的已知
空闲连通区搜索，得到其中可达的原始前沿。前沿容器不受失败目标冷却影响；
选点继续使用 Dijkstra 评分；观察位置由控制模块统一选择，再由二维和层析 A*
独立规划。普通规划断连会失败并计入脱困触发条件。

`global_frontier_valid` 表示机器人附近找到了已知空闲起点；
`global_reachable_free_cells` 是 $|R_{\mathrm{free}}|$；
`global_frontier_cells` 是 $|F_{\mathrm{raw}}|$。
只有起点有效、原始前沿数持续为零、点云与位姿新鲜、地图未满、没有活动目标，
且配置要求的 SCAN 障碍图新鲜，持续 `no_frontier_hold` 秒后，才报告
`exploration_complete`。计时结束时会不依赖索引再扫描一次完整累计地图；
若发现遗漏前沿，就修复索引并继续探索，不报告完成。出现新前沿时立即取消完成计时。
`indexed_frontier_cells` 是容器内全部原始前沿格数，`frontier_index_repairs`
是完成前独立复核发现并修正索引遗漏的次数；`global_frontier_cells` 仍只统计
机器人当前已知空闲连通区外缘的原始前沿。

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
参与规划前的筛选；已下发目标的执行交由 SCAN 负责。

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
用于区分地图断连、目标冷却与起点周围已观察空闲格的数量。无可选前沿，或同一
同一物理区域反复出现选点或规划失败时，状态机进入 `recovery`。
`recovery_phase` 依次显示 `nearby`、`turning`、`history` 或 `blocked`；
`recovery_failures` 显示当前区域失败次数。脱困不会因暂无新增观测而重复选择
同一个失败位置，也不会把仍存在的原始前沿误报为已探索完成。
到达目标后若观测增长不足，该目标进入低收益集合：先选其他正常候选；没有正常候选时，立即
从低收益前沿按前沿簇规模与路径代价重新评分选点，不等待地图新增格子。
刚到达的低收益目标仅按 `low_yield_retry_cooldown` 短暂冷却，以免原地重复选择；
规划或执行失败的目标仍按 `failed_frontier_cooldown` 冷却。使用低收益目标时，
发送状态的 `detail` 带 `low_yield`。
因此仍有前沿但都不可用时，节点会报告未解决的前沿状态，不会宣布探索完成。
没有普通前沿但可达范围内还有未知格时，节点尝试在已观测、连通的空闲格上选择补充观测点；
若当前找不到可用观测位置，状态进入脱困或报告未解决前沿；
`unresolved_unknown_cells` 给出本轮尚未覆盖的可达未知格数。
它不等于永久不可达格数，也不会触发误报完成。
`excluded_unknown_cells` 是配置矩形内、不属于当前潜在连通区域的未知格数量；
其中可能包含障碍膨胀覆盖或已知障碍隔开的格子，不能仅凭这一计数逐格判定
未知格的物理成因。
`map_capacity_reached` 表示体素上限已到，需增大容量或重启探索。
`route_source` 表示当前选用了 `layer`、`occupancy`、`recovery` 或
`recovery_unverified` 路线，`route_stages` 和
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
g++ -std=c++17 -Icpp/include cpp/src/single_floor_exploration.cpp test/test_single_floor_exploration.cpp -o /tmp/legbot_exploration_core_test
/tmp/legbot_exploration_core_test
PYTHONPATH=python python3 -m unittest discover -s test -p test_truth_coverage.py
```

## rviz路径颜色显示
[exploration 路径发布](/home/likai/legbot_3D_Nav/src/legbot_exploration/cpp/src/exploration_route_dispatch.cpp:80)现在按**实际选中的规划来源**分别发布路线；[RViz 配置](/home/likai/legbot_3D_Nav/src/legbot_exploration/rviz/multi_layer_map.rviz:267)显示二维 A* 为蓝色、层析图为橙色、脱困为黄色。切换或清除路线时，旧颜色的路径也会清空。原有候选路径话题仍可用于诊断。


## 清除残留进程
```
# 在新终端执行：
pkill -INT -f '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch legbot_exploration cross_layer_test\.launch\.py([[:space:]]|$)'

# 等几秒，让 ROS 2 launch 关闭子进程。然后检查是否还有残留：
pgrep -af 'cross_layer_test\.launch\.py|gz sim|rviz2|combined_exploration_cpp|scan_planner_node|scan_go2_cmd_adapter|scan_waypoint_mission'
```

## 查看单层探索结果
```
likai:~/legbot_3D_Nav$ ros2 topic info /exploration/status
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 0

#查看单层探索结果
ros2 topic echo /exploration/status
```
## 在线 PCT 接入

组合启动默认使用项目 PCT 作为普通层析规划后端，统一参考高度见
`legbot_bringup/config/robot_geometry.yaml`。固定世界格按共享表面记录增量融合 G/C，未知和机体空间独立处理；构图不依赖 GPU。依赖、
资源上限、XYZ 执行及验证方法见 [在线 PCT 说明](docs/online_pct.md)。

### PCT 前沿选点

`config/exploration.yaml` 新增 `frontier_source`，可选 `pct` 或 `occupancy`。当前组合启动配置使用
`pct` 配合已有的 `planner_mode: layer`；前沿提取、表面聚簇、XYZ 观察点和完成判断均取自 PCT。
详细语义、诊断话题及测试范围见 [PCT 前沿选点](docs/pct_frontiers.md)。

### 脱困请求与委托

控制模块统一选目标与动作；脱困规划验证路径/动作或返回明确的 SCAN 委托结果，路径处理模块统一下发和关联回执。详见 [执行职责与脱困动作](#执行职责与脱困动作)。恢复目标使用普通安全点，`recovery_goal_xy_tolerance` 默认0.15米，仅传给最终位置路点；普通观察目标最终路点采用 `observation_goal_xy_tolerance: 0.15`，中间路点沿用SCAN默认0.40米；同层最终观察位置距机体至少 `observation_min_distance: 0.50` 米。旧停车圆和恢复目标距离上下限已移除。详见[按路点容差](docs/goal_tolerance.md)。

旧 `/scan/waypoint_input` 的 PoseStamped 手动输入接口仍保留；探索输出使用带目标身份和可选容差的 GoalRequest。

公共楼梯识别、逐级安全站位和两种启动模式见 [公共楼梯识别与逐级观测](docs/stair_recognition.md)。
普通楼层探索只保存上升候选，不执行楼梯确认、换位或补扫；本层首次与复查队列收尾后再确认上楼。
跳过起始层仍可提前使用同一识别流程。按楼层选点的续算间隔默认 0.10 秒，累计预算继续为 2 秒／4 秒。
