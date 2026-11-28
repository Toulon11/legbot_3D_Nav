# 实验结果

`cross_layer_test.launch.py` 默认启用记录器。每次运行仅在包的 `result/` 下生成一个唯一命名的 JSON 文件；每秒原子更新，正常完成、中断和超时均保存。`.reference_cache/` 是共用场景参考缓存，不是单次实验输出。原有历史试验文件不会自动删除。

四楼测试输出格式为 `schema_version=5`。每次测试只输出一个独立 JSON，`floors` 数组分别记录 `floor_1`～`floor_4` 的探索结果；`metrics` 保留整次试验的规划和上楼指标，不输出原始事件或轨迹。

`floors` 中每层包含：楼层名、高度、状态、是否收尾、起止仿真时间，以及以下面积与探索耗时字段。下表汇总两类字段，面积和探索时间位于 `floors` 各层记录，其余指标位于顶层 `metrics`：

| 字段 | 指标与单位 |
| --- | --- |
| `reference_area_m2` | 对应楼层待探索地面投影面积，m² |
| `observed_area_m2` | 收尾时该层参考范围内已观测地面投影面积，m² |
| `coverage_percent` | 已观测面积 / 参考面积 × 100，% |
| `planning_success_count` / `planning_attempt_count` | 路径规划成功次数 / 总次数 |
| `planning_success_percent` | 规划成功率，% |
| `planning_mean_ms` / `planning_p95_ms` | 全部已结束请求的平均耗时 / P95，ms |
| `upward_success_count` / `upward_attempt_count` | 上楼成功次数 / 总次数，仅计向上任务 |
| `upward_success_percent` | 上楼成功率，% |
| `upward_mean_sim_seconds` | 成功上楼任务的平均跨层仿真耗时，s |
| `exploration_sim_seconds` / `exploration_wall_seconds` | 对应楼层普通探索仿真 / 墙钟耗时，s |

`metric_labels` 与 `floor_metric_labels` 提供中文名称，`metadata` 保留方法、场景、源码/参考哈希及关键评测配置。
`accounting` 保留上楼失败、中断、尚未结束任务数与规划耗时样本数，供核对分母。
无独立参考时面积和覆盖率为 `null`；无尝试时成功率为 `null`，无成功上楼任务时平均耗时为 `null`。
规划请求缺失任何有效耗时样本时，平均值/P95均为 `null`，避免只统计有计时的成功请求。

面积按唯一 XY / 实测地面高度计数，同一地面的切片别名不重复计数。各层只使用该层独立参考地面的范围；开始探索前提前扫到的该层地面计入覆盖面积。读取参考不意味着启动计时。
每层计时由控制器的 `pct_floor.ordinary_started_sim/wall` 开始，在首次 `ordinary_ended_sim/wall` 收尾时冻结；与当前位姿所在的独立参考地面及调度高度核对，`pct_floor.number` 不能作为建筑楼层号。
计时包括该层选点、规划、行走、观测及等待，排除前后跨层和返航。各层分别冻结面积和时间，后续扫描、任务重开或返航不改写已收尾结果。
`floors[].state` 的 `settled` 表示本层任务收尾，不表示100%覆盖；`skipped` 表示跳过普通探索；`not_started` 表示尚未开始。后两类的覆盖率、已观测面积和耗时为 null，参考面积仍可保留。中断、超时或未正常收尾就离开某层时，保留其实际覆盖率/已用时间并标注未完成，尚未开始的其他楼层不受影响。
配置楼层定义且未指定 `metrics_single_floor` 时，记录器默认启用逐层窗口；显式指定 `metrics_single_floor` 仍可使用旧版单层输出。没有楼层定义时保留整场统计。不同统计范围分组比较。
规划失败与超时参与次数及耗时统计，重复回执去重；P95采用 NumPy 默认线性插值。
上楼按物理出发和目标平台持续到达统计，重规划及补扫不另增一次任务，下降返航不影响上楼成功率。
`accounting.includes_return` 描述整场任务是否要求返航，各层探索耗时均不包含返航。
目标层任务收尾、返航完成、整项任务成功分别记录；任务收尾不能当作覆盖率100%。

本场景标定楼层高度为 0、1.46、2.96、4.43 m；出生地点为 (-6.75, 9.60, 0.0)。使用 `skip_start_floor_exploration:=true` 的结果会记录该选项，一楼覆盖不与完整探索一楼的试验混比。参考场景只用于量化，不参与目标选择或路径规划。

默认流程先完成目标高度的普通探索任务，再通过已建地图的公共安全检查与 PCT/SCAN 路线返回出生点；返航不使用场景参考路线。到达目标层本身不代表任务成功。无安全返航路线时保留未完成状态，不伪造成功。

每次测试自动只生成一个独立JSON；运行中刷新同一文件，结束时保存最终状态。
手动调用 `summarize_exploration_metrics` 才会额外生成一个 `comparison.json`，不会自动生成CSV。
该工具兼容版本3、4、5，逐层文件与旧版单层文件分组。逐层覆盖率和耗时分布只汇总已收尾楼层，跳过及未完成次数另外保留；失败和中断的完整记录保留，`partial_run=true`和未结束的记录单独标记。
批量成功率采用总成功次数/总尝试次数；分组中的耗时分布是每次试验结果的分布，不是所有规划请求合并后的P95。


在线层析入口使用 `/exploration/layers/surface_updates` 的带版本覆盖增量，
不再依赖 RViz 完整点云的刷新频率。单层收尾及任务结束时，等待覆盖统计至少
处理到控制器所报 `layer_map_version` 后才冻结面积；已记录的普通探索起止时间
不因等待发布而延长。结果仍为每次测试一个 JSON，`accounting.coverage_export`
保存处理版本、epoch、同步状态和待结算数。中断时若仍未同步，保留实际统计
和未完成状态，不伪造已完成覆盖率。独立使用记录器且未指定
`surface_updates_topic` 时，保留原完整 `surfaces_topic` 兼容输入。
