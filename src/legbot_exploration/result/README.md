# 探索测试量化结果

每次测试只生成一个独立的JSON文件，固定保存在本目录，文件名由UTC时间戳和随机编号组成。
运行中每秒更新同一文件，完成、中断、超时或手动结束后写入最终状态。
不再生成时间线CSV、轨迹CSV和事件JSONL，也不输出逐楼层统计。

```bash
cd /home/likai/legbot_3D_Nav
source /opt/ros/jazzy/setup.bash
source install/setup.sh
ros2 launch legbot_exploration cross_layer_test.launch.py skip_start_floor_exploration:=true
```

`metrics` 对应专利中的量化表：二楼参考地面面积、二楼已观测面积、二楼覆盖率、路径规划成功/总次数与成功率、
规划平均/P95耗时、上楼成功/总次数与成功率、成功上楼平均耗时，以及二楼普通探索仿真/墙钟耗时。
`metric_labels` 提供中文名称和单位；百分比采用0–100，缺少参考或样本时为`null`。

上楼只统计向上的物理跨层任务，下降返航不会提高上楼成功率；同一任务中的重规划和补扫不另计尝试。
成功上楼平均耗时仅包含成功任务，成功率的分母仍包含失败、超时和中断。
规划耗时包含失败请求，P95采用线性插值；耗时样本不完整时平均值及P95留空。
默认只评价`floor_2`（1.46 m）的覆盖率及探索耗时；提前扫到的二楼地面计入面积，同一地面的多切片表达不重复计数。
从二楼普通探索开始计时，到首次交给上行调度时冻结结果，后续扫描、跨层和返航不再改变它。
`single_floor.state`记录未开始、进行中、收尾或中断；收尾不代表覆盖率100%。规划和上楼指标仍统计整次测试。
`accounting.includes_return`仅描述整场任务配置，单层耗时不含返航。

`finalized=false`表示尚未结束或被强制终止，不视为完成试验。保留失败与中断记录，不能只选成功结果。
启动后未进入有效任务的试验，耗时留空。跳过起始楼层探索的选项写入`metadata`，不能与完整探索混比。

手动结束当前量化记录：

```bash
ros2 service call /exploration/benchmark/finish std_srvs/srv/Trigger '{}'
```

该服务只结束记录，不停止机器人。`record_metrics:=false`可关闭记录，`result_directory:=...`可指定其他目录。
`.reference_cache/`为共用的独立场景参考缓存，不是历史试验结果，也不参与机器人规划。

如需批量对比，可手动运行`ros2 run legbot_exploration summarize_exploration_metrics`，只生成一个额外的
`comparison.json`，不自动生成CSV。完整字段、统计口径和参考边界见
[实验结果说明](../docs/benchmark_results.md)。
