# 按路点指定到达容差

## 原子接口与兼容

`scan_planner/msg/GoalRequest` 包含目标 PoseStamped、goal_id、attempt_id、waypoint_index、has_xy_tolerance 和 xy_tolerance（米，仅水平XY）。未指定时使用SCAN各节点配置的默认值；当前启动链统一为0.40米。指定值必须有限且位于消息常量规定的[0.05,1.0]米内，非法值明确报invalid_goal_tolerance，不静默夹紧或回退。

探索路径处理模块发布 `/scan/waypoint_request`；`scan_waypoint_mission` 在direct模式透传到 `/scan/goal_request`，沿用原有时间戳重写、执行就绪和完成策略。它为该目标记住相应回执校验范围。SCAN规划状态机与执行适配器都直接接收这条完整消息，先确定该目标容差，再处理目标。新目标每次重置有效容差。GO2执行协调器额外订阅扩展目标，仅将target交给原on_navigation_goal回调；1.5秒空闲、行走激活和站立策略不变。

扩展命令不会同时发布一份 `/scan/goal` 位姿影子，避免先到的默认命令提前停车。原 `/scan/waypoint_input` PoseStamped输入和 `/scan/goal` 链路保留，继续使用默认容差。扩展请求接口用于direct模式；旧queue模式不接收扩展请求，并明确发布request_requires_direct_mode状态。

## 归属与默认策略

控制模块通过 `PlanningGoal.xy_tolerance` 指定执行要求，规划模块保留它，PathProcessor仅在最终位置路点输出该值。普通探索最终观察目标采用 `observation_goal_xy_tolerance: 0.15`，同层最小观察距离由 `observation_min_distance: 0.50` 约束；脱困position/delegate最终目标采用 `config/exploration.yaml` 的 `recovery_goal_xy_tolerance: 0.15`。中间路点使用默认值；reverse/turn保持各自位移/转角判定。

SCAN的finish_dist全局默认保持0.40米。每次接受新目标，规划完成判断、执行停车和末端减速均使用本次有效XY容差。未指定的目标使用默认值。真机可达到的停车精度需要实测。

## 安全点与脱困结果

脱困位置目标统一使用普通point_safe_check，仍检查机体、未知、实体、坡度和净空；删除原recovery_arrival_radius和额外停车圆筛选，未缩小机体半径。盲起点明确委托SCAN，目标不被替换成近点。控制侧倾向更远的盲区退出候选，没有目标距离硬上限。

到达回执仍由SCAN提供，PathProcessor核对当前目标时间戳，只结算一次。
扩展请求先确认目标/尝试/路点编号，再接受匹配的执行器停车回执；先到的回执暂存至绑定。
不要求即时到达的目标一定产生可观察到的 Bool false 边沿；单独旧 Bool 或旧时间戳不能推进路点。最终成功后仍等待新地图，对实际机体位置复用普通起点准入；通过才退出脱困。更小的执行容差不替代这一检查。

普通层析路线将连续平缓段作为一次执行请求，reference_points 保存完整 XYZ 转折，
target 指向该段末端。协调器透传整条参考，SCAN 复用分段参考轨迹及局部重规划。
同一起始格的锚点可省去单独的停车任务。相邻高度变化及不超过75度的转折保持在连续参考中，
只在超过75度的急转和最终观察点处分段停车；跨台阶仍保留每个已验证XYZ参考点，不以简化段端点代替真实路线。
急转停车后的下一参考段包含原拐点，防止因到达容差从实际停车位置切角。最终观察点保留0.15米容差，脱困路径继续使用其独立动作。

地图提交阶段缓存已解决的前沿任务，路点交接读取原子缓存，取消已解决任务的剩余路线；
续段不再同步触发地图提交、Python查询或地图锁等待。重新尝试同一簇会清除旧完成缓存。
日志 `Route handoff ... handoff_ms=...` 记录停车回执处理后至下一段下发的交接耗时。
执行端减速、真实速度门槛和0.30秒连续稳定确认保持原值。
reference_points 为空时沿用单点请求，非空时要求至少两点、所有坐标有限且末点等于 target。
接口更新后需完整重启探索、SCAN 和执行协调器。

## 诊断与测试

`/scan/goal_tolerance_status`：JSON，source为planner/executor，event为accepted/rejected/arrived。包含目标身份、时间戳、是否指定、请求值和实际采用值；executor的arrived事件额外报告实际位置与目标距离。它是执行诊断，不能用另一条容差话题改变正在执行的目标。

`/exploration/execution_feedback`继续报告统一成功/失败，增加requested_xy_tolerance（null表示未指定）、成功停车回执的actual_position和goal_distance。

`scan_goal_tolerance`测试启动真实协调器/执行器，验证0.30米外的目标在0.15米要求下继续运动、进入范围才确认、实际停车位置、下一目标恢复默认、旧目标忽略、非法值拒绝和旧PoseStamped兼容；同时启动真实SCAN规划状态机验证同一请求的接受/默认/拒绝。里程计和轨迹为合成数据，不等同于Gazebo或真机运动验证。
