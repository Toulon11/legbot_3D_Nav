# PCT 按层增量前沿与观察目标

当前生产控制采用[三维局部选点优先、全局后备](pct_local_window.md)。
以下增量索引同时提供全局统计与局部查询；局部为空不会被当作全图探索完成。

失败记录按物理边界保留。附近已有地面的代价跨阈值或新增同 G 切片不会重新开放失败簇；
只有附近新增可用的实测地面覆盖，才增量重开可能因缺测而失败的任务。首次可用地面
按同列实际高度去重，避免同一块地面反复变为可通行时重置首轮和复查计数。
有限方向搜索完整结束且所有已检查站位均被已知表面遮挡时，直接按已知阻断结算，
不再进入第二轮。前沿证据仍保存在地图中，调度结算不删除真实地图记录。

## 统一的切片已知性与前沿入口

构图和前沿采用[同列实测开区间规则](slice_intervals.md)。每层的前沿成员只有一个生成条件：
本层 XY 经统一同列查询为未知，八邻域至少有一个通过同 G 层辅助高差检查的
已知非障碍格，并且八邻域没有已知实体障碍。即使同时邻接自由格和已知障碍，也
排除该候选。未知膨胀造成的停车不安全不等于已知实体障碍，不能据此抹去全部探索边缘。
辅助信息限制在来源格同一地面对应的切片内，高差上限为 16 cm；
没有辅助信息时保留未知前沿，具体规则见 slice_intervals.md。
同列其他切片的已知 G/C 严格包住当前切片高度时，该 XY 已知，无需创建实体邻格。
同一 XY 已有同高实测地面的工作格时，不因切换切片重新生成该地面的前沿，包含
`C=+inf` 的开放地面。只结算这份地面，不把无穷上界扩展成其他独立楼层的已知自由空间。
来源按地形代价 `[0,20]` 判断；停车足迹、未知膨胀、盲区和路径检查在观测位姿选择时执行。
Python 与 C++ 边界内核使用同一份“已知标志＋实体障碍标志＋合格来源”输入，并共用辅助高差过滤，
不再用回波接近高度、
台阶边界推断或补扫预期分别消除前沿。簇失败与复查只改变任务调度状态。


## 配置

```yaml
frontier_source: pct
planner_mode: layer
observation_min_distance: 0.50
observation_goal_xy_tolerance: 0.15
```

使用同进程在线 PCT，不使用二维占据图或 SCAN 障碍点生成 PCT 前沿。
二维选点仍可通过 frontier_source=occupancy 启用；普通二维观察目标也携带指定最终容差。

## 选点续算与耗时

按楼层调度时，未完成的选点、已结算后继续下一个簇、局部转全局查询，共用控制器原选点入口续算。
每批返回并释放地图锁后，默认等待 `pct_query_resume_delay_seconds=0.10` 秒再接续；允许范围 0.05～1.0 秒。
执行、补扫、脱困、暂停或地图不可用时，不由快速定时器重复调度。非楼层旧模式保留原定时流程。
当前普通簇启用四方向搜索，累计搜索兜底为 1.5 秒；第二轮默认关闭。
兼容的非四方向接口保留 2 秒／4 秒上限，不能混作当前首轮配置。

平层任务按 `(1 + 距离)/sqrt(min(待观测物理格数,64))` 估算每次访问的成本，优先附近且覆盖更多的任务。
候选位姿也按距离/地形成本与实际可见格数排序；所有大小的簇仍参与，不添加转向成本。
上升簇的既有优先级保持独立。站位仍须通过完整停车、盲区、视线与路线检查。

软代价变化但候选高度、表面身份和可通行状态不变时，不撤销候选几何缓存；查询按地图块批量读取最新代价。
跨越通行阈值、支撑高度改变或新遮挡出现时仍失效。位姿视线缓存不依赖机器狗当前所在的格号；
当前位置只用于本轮距离过滤、连通性和排序，各站位仍检查其安全签名及射线依赖是否变化。

状态增加 `pct_selection_timing_ms`、`pct_observer_prepare_ms`、`pct_selection_resumes` 和
`pct_selection_resume_gap_ms`。计时区分读锁等待/提交、楼梯任务、前沿查询、起点检查、停车范围、盲区、视场、遮挡和路线。
前沿查询包含部分子阶段，计时有包含关系，不能直接相加。每个选点周期结束后记录完整的 `PCT selection timing` 日志；
状态发布在周期中途时，尚未结束的阶段可能在下一次状态中才显示。

## 前沿放在未知侧

前沿记录紧邻已观测表面、但尚未取得对应高度实测证据的位置。
索引键为固定世界切片编号、XY 格编号和来源表面身份；同一切片、同一 XY 可以保存多个高度任务。
表面身份包含来源列 XY 和列内表面编号，不能把不同列中恰好相同的编号当成同一表面。
节点保存参考地面高度及邻接已知表面的来源、代价。
参考高度只用于分层、聚簇和观察目标选择，不向地图写入推断 G。

每列多个高度地面分别保留；同一地面的多个切片和不同代价状态也分别保存。
取得对应高度的实测回波后移除该前沿，保留同列其他尚未观测的高度。已知阻挡不产生未知前沿。
全未知列只为邻近实际表面涉及的切片创建前沿，不铺满任意高度或全图未知空间。
每个切片内分别对来源高度分组，不再只取最高的一组；分组跨度受 surface_noise_tolerance 限制，
不允许通过一串相近高度无限串接，也不允许同一来源列的不同表面编号进入同一个组。
固定切片没有选为 G、但真实成熟且有足够上方空间的表面，也保留待观测边界；这不会创建新的规划格或安全代价。

普通前沿已知/未知只使用同列有效实测区间（详见 slice_intervals.md）。
所有回波均参与 G/C；开放上界共享限制为 G 上方 5 米。区间已覆盖的格禁止生成普通前沿，
邻接踏面 G 不同不能覆盖这个已知判定。具体补扫的目标 G 和帧号验收独立保留。
不同切片中同一物理前沿共用一次探索任务。

同列障碍回波相邻高度差小于最小净空长度时，累计合并为不可通行闭区间；沿用已有
地面噪声归组。查询高度落入区间则为已知障碍、代价至少 50，即使本帧未命中或没有
实体切片格也不生成未知前沿。区间独立保存，不覆盖 G/C，不把回波间隙填成光学实体。

台阶交界只使用统一已知区间、实测占据和同 G 辅助高差检查，不另用整梯段或九格
边线识别作为生成前沿的必经条件。未知证据保留，辅助高度不写入地图 G。

## 各切片独立更新

`TomographicSlice` 持有固定的切片高度、稀疏格子和本层前沿簇。点云先融合为共享列证据，
每列比较提交前后的有效表面高度、表面编号和成熟状态，按这些表面的前后相邻高度确定
G/C 可能变化的切片区间，再把“切片、XY、对应 G/C 表面”组成批量更新任务。
新增切片从该列完整累计证据初始化。表面均值移动、合并或被噪声处理移除时，
同时检查变化前后的区间。各切片独立更新 G/C、地形代价和膨胀。
这些任务都结束后，再更新各层前沿及簇。地图、前沿和连通性准备完成后统一提交版本。
成功提交后才保存列的高度序列和已初始化切片编号，更新失败后重建这些记录。
诊断统计提供 `geometry_candidate_cells`、`geometry_update_cells` 和
`geometry_skipped_cells`，分别表示原本涉及的切片格、实际派发格和省去的格。

更新前沿时比较变化列的潜在地面、来源身份和实测高度。只有影响前沿的变化才重查本列及八邻域。
G/C 未变化的列也继续参与前沿证据检查，确保后续回波能够消除未知前沿。
每层用成员哈希表和成员到簇的反向索引处理本批新增、删除和连接变化。
邻接未变化时只刷新属性；纯新增按接触到的旧簇加入或合并。
删除或高度变化导致连接丢失时，先在变化区外扩两格内检查剩余连接点；
局部已证实仍连通则保留旧簇，否则仅遍历受影响旧簇，连接点全部到达时提前结束。
批量删除使用本批最终状态，簇 ID 优先由重叠最多的部分继承。
同簇成员按空间块分组供查询使用；分组内容不变则保留原标识及观察位姿缓存。
候选站位的安全签名独立跟踪。

提交时仅对前沿变化或所属簇变化的 XY 更新跨切片别名，按同高度和来源身份相容性去重。
用这些位置的别名联系更新受影响的物理任务及空间索引。查询直接读取已提交的索引。
任务内部保留切片引用，各切片的原始前沿和通行代价仍可独立查询。
失败与复查按物理前沿记账。

`pct_slice_update_workers` 默认 `0`，按每批约 8192 个切片格的工作量切换到最多 4 个线程。
设为 `1` 可串行回放。G/C 写入、地形代价和逐格前沿计算使用释放 GIL 的 C++ 批处理。
点云融合、簇连接、跨切片备选关系维护和统一提交仍包含 Python 工作。

观察位置从附近约 2 米范围内的原地图分块查询，要求在线安全代价在 [0,20]。
地图提交时不搜索观察位置；控制器查询时按需建立“簇的空间块分组 × 切片”的候选几何缓存，
保存候选位置、附近前沿距离及代价，而不是只保存三个最终点。
不再对整个簇抽取64个种子；每个局部片的全部前沿参与距离判断，计算按批次限制临时数组大小。
真实表面变化只使依赖对应切片、候选块及前沿半径范围的缓存失效；空间块分组改变时清理其旧缓存。
机器人位置、方向、连通分量和已完成观察位置在查询时过滤，随后选最多三个候选。
移动或完成观察不会撤销几何缓存；远处地图版本变化也不直接使全部候选失效。
三维窗口裁过块内片时，复用安全候选，但只计算窗口内前沿的距离，避免把窗口外前沿记为本地收益。
控制器读取索引、重新评分，不调用全图提取。
删除唯一连接点仍可能遍历受影响簇成员；元信息汇总也可能涉及该簇成员。
大簇更新在一次地图提交内完成，成本由连接检查、重建成员数和耗时单独记录。
完整重建入口仅用于离线对照测试，生产路径不调用它。

## 同一版本、零地图快照

控制器在选点前申请 LayerMapReadPhase，持有至该次规划决策结束。
内部安全检查和原生 A* 借用同线程已有读阶段。写端在整个读阶段丢弃建图帧；
读阶段结束后恢复建图，提交地图及其前沿索引。不复制全图，SCAN 实时执行不暂停。

## 观察位置的距离与到达容差

同层普通探索的最终观察位置距当前机体至少 observation_min_distance，默认 0.50 米。
跨层目标保留高度区别，不仅凭 XY 接近排除。控制器在规划前和装载执行路线前检查最终位置。
参数要求最小观察距离大于合法的最终到达容差；当前指定容差默认 0.15 米。

PlanningGoal.xy_tolerance 经 PathProcessor 只附加在最终路点，SCAN 两端沿用已有 GoalRequest 接口。
中间层析路点保持原生 A* + RDP 的坐标、顺序和默认容差，不跳点、不额外简化或后验改边。
到达回执也按请求的最终容差验证；未指定时保留原有回执门槛。
带 goal_id/attempt_id/waypoint_index 的转发请求确认归属后，匹配时间戳的执行器停车回执可完成当前路点。
回执先于转发回调到达时暂存并在绑定后核对；不依赖深度一 Bool 必须看到 false→true，
避免近路点即时到达时丢失 arming 边沿。旧 PoseStamped 输入保留原有 Ack/Complete 兼容流程。
脱困、历史回退和转向不套用普通观察位置的 0.50 米限制。

## 完成观察与任务完成

最终观察位置成功后，索引按空间位置及实际高度记录已观察位置，排除附近重复观察位置。
记录独立于簇编号及整簇版本：同一大簇远端变化、其他楼层变化不清除该位置；
静态图普通前沿扩展和软代价变化不清除已完成记录；相关安全签名变化时复查该位置，
仅实际安全条件失效或支撑高度实质改变才撤销。邻近候选仍安全或某个等价切片变化不会直接撤销。
不会仅因八秒冷却结束再次下发同一个位置；同表面相近视角默认排除半径为1.0米。
全局其他区域更新不应清除该记录。未启用按楼层调度时，执行失败的目标仍按失败冷却处理。

普通楼层探索启用 `pct_floor_exploration` 时，失败按物理前沿成员记账。
所有大小的簇均可参加首轮；完整全局搜索找不到观测站位的任务退役，
缺测、超时、执行失败及实际观测进展不足保留对应失败原因。
本层局部和全局首次任务结束后，仅在显式启用第二轮时全局复查；当前实验关闭二轮。
当前四方向首轮累计搜索上限 1.5 秒；找到一个合格观察站位就交给路线检查。
这些路线均失败或本轮规划预算耗尽，就结算本次失败，不持续生成下一批站位占用同一轮尝试。
遇到可补扫的未知阻断时，先交给已有定向脱困；该轮仍失败后再统一结算。
簇重编号、分裂、切片别名或普通地图版本更新不清空成员的失败记录，其他楼层不受影响。
放弃仅移出调度队列，底层实测地图与未知前沿证据仍保留。

同层连通性粗筛只证明能否到达观察站位。确认整个当前任务均不连通时，立即结束本次
搜索，省掉视线检查和 A*，交由控制器按上述首次／复查规则结算；首次不永久剔除。
局部窗口内的一部分不可达，若整个任务仍可能存在可达站位，只跳过局部部分并交给全局查找。
全局查询结束、首次与复查队列均空且无正在执行或续算的任务时，立即交给找楼梯，不额外等待。
`pct_disconnected_floor_frontier_cells` 统计复查仍不连通而最终放弃的格数；首次不连通
通过 `floor_disconnected` 结算状态及待复查队列体现，不提前计入最终放弃数。

以下 `pct_region_complete` 条件用于未启用上述按楼层调度的模式。
地图无效、没有安全表面、起点不安全、候选冷却、观察位置耗尽或规划断连都不是探索完成。
只有地图健康、起点安全、待观测前沿为空并保持 no_frontier_hold，才报告 pct_region_complete。
它不证明整栋楼各层都已经探索。已保留但暂无新观察位置的前沿报告 pct_frontiers_without_new_observation。

## 诊断与验证

/exploration/status 保留前沿数、簇数和规划状态，另显示两个观察目标参数。
在线地图 metadata.frontier_index 报告 checked_columns、changed_frontier_layers、
topology_changed_layers、boundary_input_columns、relabelled_tiles、reclustered_layers、
component_nodes_visited、affected_clusters、cluster_rebuilds、local_connectivity_checks、
local_connectivity_preserved、physical_columns_checked、physical_clusters_updated、physical_tasks_ms、
observer_invalidated、updated_clusters、update_ms、frontier_layers 和 clusters。
reclustered_layers 现在指实际重新进行块内连通标记的层格数，不再是整个受影响簇大小。
Python查询结果另有 observer_cache_built，便于区分按需缓存构建和候选汇总耗时。
/exploration/pct/frontiers 显示未知边界、已知安全观察位置及当前选中目标。

回归比较每批局部更新与相同未知侧定义的完整重建：前沿成员、簇连接及观察候选一致；
覆盖单层成熟、多层未知保留、簇合并/分裂、代价变化、多批提交、不扫描全图及完成记录保持。
新增同切片多高度、14.3厘米底面/踏面、单条模糊观测不清除两项任务、
1.4厘米相邻台阶连续性以及固定切片未选中的真实表面回归。
另由 ROS 测试验证目标距离、指定最终容差、原样路点、匹配执行回执和不误报完成。

### Observation tasks (2026-11-08)

Observer candidates now predict the auxiliary LiDAR field of view using the same
mount/FOV parameters as the simulation (`legbot_bringup/config/robot_geometry.yaml`).
The prediction assumes the planned body is level, transforms frontier XYZ into the
sensor frame, and checks azimuth, elevation, range and known surface occlusion.
It is a visibility estimate, independent of the ordinary point/path safety check.
Gain is the visible local frontier count, rather than the entire component size.

Observation admission traces only the stance-to-target XY projection for occlusion.
Every touched cell, including diagonal corner contacts, is checked for a measured
obstacle in the stance's ground space. Shared same-column intervals and occupied
bands participate, including walls with no standable G. Independent floors and
confirmed short risers ending at measured upper treads are not same-floor walls.
A 3-D optical ray passing above an obstacle cannot admit that view. Range, FOV,
blind-area and parking checks remain; movement keeps its own collision checks.
Changes to obstacle queries invalidate cached views. Targets are actual visible
members; one hidden member does not exclude another clear part of the cluster.

Position arrival starts a normal exploration observation task. At the actual
stopping pose the controller recomputes the view and requests checked yaw actions.
The controller keeps the current heading if it covers at least 80% of the best
predicted view. It dwells for 1 second at the primary heading and can add one
side view at ±0.35 rad. The side view must add at least two frontier cells and
at least 20% of the primary view's coverage. The task ends at the last heading.
Gain and freshness remain diagnostics. Unknown frontiers remain indexed until
actual measurements resolve them. The mapper continues integrating during motion
and dwell.

Completion stores actual position, yaw coverage and the associated frontier keys
and heights. Position-only legacy records remain supported by the Python test API.
New direction-aware records do not exclude a view merely because its XYZ is near
an old position. `/exploration/observation_feedback` retains task outcomes and
`/exploration/status` exposes `observation_task_active`, desired yaw and evidence.

Normal turns use the existing stamped action transport (`/scan/recovery_turn_request`
and its result/cancel topics); the topic name is retained for compatibility. Their
planner kind/source is `observation_turn`, and control mode remains `flat`.
The same circular-footprint geometry check covers all headings; the execution
adapter and sequencer retain exclusive ownership of velocity and gait modes.

Set `observation_flat_frame_wait` for normal views (1 s by default) and
`observation_frame_wait` for stair views (2 s). The overall deadline is
`observation_task_timeout` (60 s) and the side angle is
`observation_sweep_half_angle` (0.35 rad). The optional
`observation_full_sweep_fallback` adds one full turn when explicitly enabled.
A failed motion ends the task with its execution reason.

Large normal-observation rotations are serialized by PathProcessor into at most
one-radian actions, each with the existing executor watchdog. Observations are
armed only after the last action. No mode or velocity publisher is added.

Observation gain/fresh height association also uses surface_noise_tolerance;
a nearby underside within step_max cannot count as a measured upper tread.
The finite sweep completion policy remains independent of these diagnostics.

The mapper's height-group fusion is unchanged: returns in the SAME XY within
surface_noise_tolerance can still share one record. Frontier indexing cannot
recover a thin top/bottom pair already merged by that input rule. This update
does not classify surface normals or establish successful stair traversal.

Observer queries use pct_query_budget_seconds, default 0.20 seconds. Clusters
are ordered by coarse distance and frontier size before observer evaluation.
Unfinished visibility searches resume across controller ticks, and pending
queries defer global fallback. Frontier display entries remain complete.
Visibility masks cache the mature surface heights of the columns actually
visited by their rays, including empty columns. New or changed mature evidence
invalidates the mask. Status exposes query_pending, checked-cluster count,
observer geometry time and visibility time. Budgets yield between complete
candidate checks, so an individual check can exceed the configured slice.

## 墙体跨切片过滤与低收益暂缓

前沿资格和观测站位到目标的格线检查共用按实际支撑高度查询的墙体占据判定。
实测占据区间与支撑高度以上、单步能力到最小净空之间的空间相交时，不能因
当前高切片缺少 G/C 就重新生成该墙的未知前沿或隔墙观察。该区间来自真实回波，
不把房间的自由 G/C 区间当作实心障碍。低楼层天花板位于新平台地面以下时，
不会阻止新平台前沿。墙体区间变化也进入前沿增量签名，及时结算高切片旧成员。

当前实验配置保持 `pct_fast_exploration: true`，关闭 `pct_floor_global_review_enabled`。
所有大小的簇都参加首轮，至少一格可见就可以选点和观察。不能因成员少于 3 或预计
收益小于 3 直接跳到复查。真正观察后才按实际解决格数结算：解决全部单格成员算成功；
有新帧但实际进展不足记为 `floor_low_progress`，直接进入复查队列，关闭二轮时不重复观察。
局部缺测/选点失败的大簇仍有全局首轮机会；全局有界搜索完成但找不到无障碍视角的任务
退出本轮调度。超时、缺测与断连独立记账，不能伪装成已确认墙体遮挡。地图证据继续保留。
只有相关新增实测安全地面可触发缺测失败重试，静态命中和切片别名不重开首轮。
旧的 `pct_low_yield_deferred_clusters/cells` 字段为兼容诊断保留，不再表示首轮尺寸过滤。

## 局部失败与全局首轮分开

普通平层簇的局部失败只进入 `floor_local_failures`，不消耗大簇的全局首轮。
全局查询从完整同层簇及其全局候选站位重新搜索，仍报告 `attempt=1`、
`reviewing=false`。首轮最多四方向与单簇搜索时限继续生效，不开启第二轮复查。
全局首轮也失败后，才进入原有暂缓/复查记录；关闭第二轮时不再重复调度。

小簇按完整同层簇的唯一 XY 数量判断。局部窗口只看到大簇的一小段时，可以
暂缓这一段局部选点，不能因此把整个大簇的全局首轮跳过。真正少于三个成员
的小簇也参加首轮，实际观察进展不足才进入复查。局部尝试只结算本窗口实际参与的成员，并在
异步行走和观测期间保留自身的局部/全局来源，避免后来的查询改变结算范围。

局部受限站位全部被遮挡，只证明局部选点失败；大簇仍须接受全局首轮检查。
全局已确认障碍的退役规则、公共停车/盲区/视线/路线安全检查继续保留。
全局搜索复用局部已被路线检查拒绝的站位记录；仅因局部视野不足而试过的
站位，仍可针对完整簇重新计算可见收益，不能全部当成坏站位排除。
相关新增实测安全地面可以增量重开两类失败，静态重复命中不重开。

控制状态中的 `pct_global_first_after_local_failure_clusters/cells` 表示当前全局
首轮候选中带有局部失败历史的簇/物理成员数，便于核对收尾前是否提供了
扩大范围的搜索机会。这些数是单次查询的候选量，不是累计成功次数。

观测站位由物理地面而非簇所在切片限定：主切片的簇可以从其他切片中同高的实测支撑上观测。候选按 XYZ 去重，缓存保留实际来源切片的代价和依赖。它不会把室外 C=∞ 外推成高切片已知；公共停车、盲区、视线和路线检查继续执行。

起点因新观测或位置偏差进入膨胀区时，即使已无待扫目标，也允许先查询 0.25～0.60 m 的短移恢复。必须已有实测支撑，实际高差不超过16 cm；不新增足迹冲突，距离原障碍不得减小，提前刹车位置及终点均须安全。执行停止回执与实测位置确认后，恢复普通探索或返航。失败的 SCAN 委托释放动作状态，不能永久屏蔽短移恢复。
