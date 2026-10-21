# Legbot FAST-LIVO2 接入

`fast_livo` 提供两份单文件 ROS 参数配置：`config/legbot_sim.yaml` 对应 GO2 Gazebo 仿真；`config/avia.yaml` 保留原始 FAST-LIVO2 `avia.yaml` 和 `camera_pinhole.yaml` 的参数，用于 Retail_Street bag。`launch/legbot_sim_livo.launch.py` 默认使用仿真配置，仅启动映射节点和可选 RViz。`launch/mapping_aviz.launch.py` 默认使用 `avia.yaml`，并沿用上游 `fast_livo2.rviz` 和压缩图像转原始图像的节点。FAST-LIVO2 的 `vk::camera_loader::loadFromRosNs` 从所选 YAML 的 `camera` 命名空间读取 `cam_model`、`cam_fx` 等参数，无需单独的相机 YAML 或 `parameter_blackboard` 节点。

## 构建和启动

```bash
cd /home/likai/legbot_3D_Nav
./tools/build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.sh
ros2 launch fast_livo legbot_sim_livo.launch.py use_rviz:=true
```

测试原始 Retail_Street bag 时，使用上游的启动命令；该 launch 现在会读取合并后的 `avia.yaml`：

```bash
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True

# 另一个已 source 工作区的终端，进入 bag 所在目录：
ros2 bag play Retail_Street --clock
```

`mapping_aviz.launch.py` 默认启用压缩图像转换，订阅 `avia.yaml` 中 `common.img_topic` 对应的 `/compressed` 图像，再发布到原始图像话题。若 bag 已提供原始图像，可加 `republish_compressed:=false`；自定义配置路径用 `avia_params_file:=...`。RViz 使用原始的 `rviz_cfg/fast_livo2.rviz`，固定坐标系 `camera_init` 与 `avia.yaml` 的 `common.world_frame` 一致。

播放前用 `ros2 bag info Retail_Street` 检查 `/livox/lidar` 的消息类型；如果转换后的元数据仍写着 `livox_ros_driver/msg/CustomMsg`，应按上游 README 的示例调整为本工作区的 `livox_ros_driver2/msg/CustomMsg`。本工作区未包含 Retail_Street bag，完整回放仍需用实际数据验证。

订阅话题在两份 YAML 的 `common.lid_topic`、`common.imu_topic`、`common.img_topic` 中配置；所有显式发布的话题在 `output_topics.*` 中配置。Legbot 配置直接将里程计和注册点云发布为 `/fast_lio/odometry_sensor`、`/fast_lio/cloud_registered`，启动文件不再做固定 remap；原始 bag 配置保留上游 `/aft_mapped_to_init` 和 `/cloud_registered`。话题名在节点启动时读取，修改 YAML 后重启节点即可生效；安装目录不是源码软链接时需先重新构建 `fast_livo`。

使用 Legbot 配置时，启动前应由仿真程序及桥接节点提供 `/clock`、`/livox/points_raw`、`/livox/imu`，运行视觉融合时还要提供 `/camera/image`。此 launch 不启动 Gazebo、GO2、相机、桥接或里程计适配器，也不要与原 FAST-LIO 同时运行。`use_rviz:=true` 只控制 RViz。默认 `common.img_en: 1` 表示 FAST-LIVO2 使用图像；将它设为 `0` 可在没有相机话题时测试 LiDAR+IMU，但该参数不会创建或关闭 Gazebo 相机。

映射节点将原始里程计发布为 `/fast_lio/odometry_sensor`，点云发布为 `/fast_lio/cloud_registered`。需要供导航使用的 `/fast_lio/odometry_base` 时，应另行启动 `fastlio_odom_adapter`。Gazebo 仿真的前视相机由 `legbot_bringup/simulation.launch.py` 的 `enable_camera` 控制，默认 `true`；使用 `enable_camera:=false` 可关闭相机传感器。图像和内参经 `ros_gz_bridge` 发布为 `/camera/image` 与 `/camera/camera_info`。

## 标定参数读取

仿真运行时另开终端：

```bash
cd /home/likai/legbot_3D_Nav
source /opt/ros/jazzy/setup.bash
source install/setup.sh

ros2 run fast_livo report_calibration.py
```

脚本从已安装的 GO2 模型计算 LiDAR→IMU 的 `extrinsic_R`、`extrinsic_T` 和 LiDAR→相机光学系的 `Rcl`、`Pcl`；订阅 `/camera/camera_info` 读取当前图像尺寸、内参 `K` 和畸变系数 `D`。只读取外参时加 `--extrinsics-only`；自定义等待时间可用 `--timeout 20`。脚本只打印结果，不自动改 YAML。相机参数若发生变化，应将 `CameraInfo` 中的数据同步到 `config/legbot_sim.yaml`。

手动查看话题也可以用：

```bash
ros2 topic echo /camera/camera_info --once
ros2 topic hz /camera/image
ros2 topic hz /fast_lio/odometry_sensor
```

当前仿真的 `CameraInfo` 为 1280×720，`fx=369.67277526855469`、`fy=369.67273235321045`、`cx=640`、`cy=360`，五个畸变系数均为零。`camera.scale: 0.5` 让 FAST-LIVO2 同比缩小图像和内参。雷达点云使用 Gazebo XYZ/I 输入，因其没有逐点时间戳，按 10 Hz 扫描周期估计点相对时间，仅用于仿真。
