/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

/**
 * @file LIVMapper.h
 * @brief FAST-LIVO2 核心映射器类，整个系统的顶层协调器
 * 
 * LIVMapper 负责：
 * 1. 多传感器数据缓冲与时间同步 (LiDAR/IMU/图像)
 * 2. 主循环控制流：sync → IMU → LIO → VIO
 * 3. 管理所有子模块：Preprocess、ImuProcess、VoxelMapManager、VIOManager
 * 4. ROS 消息发布 (里程计、路径、点云、图像)
 * 
 * 设计模式：作为“门面”(Facade)，将复杂的多传感器融合流程封装在一个类中
 */
#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vikit/camera_loader.h>

/**
 * @brief 核心映射器类，管理整个 LIVO 系统的运行
 * 
 * 主要工作流:
 *   run() → sync_packages() → processImu() → stateEstimationAndMapping()
 *                                         ├→ handleLIO()  (LiDAR点到平面匹配 + EKF更新)
 *                                         └→ handleVIO()  (视觉直接对齐 + EKF更新)
 */
class LIVMapper
{
public:
  LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name);  // 构造函数：读取参数、初始化子模块
  ~LIVMapper();
  void initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr &nh, image_transport::ImageTransport &it_);  // 初始化ROS订阅/发布
  void initializeComponents(rclcpp::Node::SharedPtr &node);  // 初始化子模块组件
  void initializeFiles();  // 初始化调试输出文件
  void run(rclcpp::Node::SharedPtr &node);  // 主循环：等待数据、调用同步和处理
  void gravityAlignment();  // 重力对齐（初始化阶段）
  void handleFirstFrame();  // 处理第一帧的特殊逻辑
  void stateEstimationAndMapping();  // 状态估计与建图的主调度函数
  void handleVIO();  // 视觉惯性里程计处理
  void handleLIO();  // LiDAR惯性里程计处理
  void savePCD();  // 保存点云地图到文件
  void processImu();  // IMU数据前向传播
  
  bool sync_packages(LidarMeasureGroup &meas);  // 多传感器时间同步，将LiDAR/IMU/图像对齐
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);  // 单步IMU传播
  void imu_prop_callback();  // IMU传播定时器回调
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);  // LiDAR点云坐标变换
  void pointBodyToWorld(const PointType &pi, PointType &po);  // 单点体坐标系→世界坐标系
  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);  // LiDAR体→IMU体
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);  // 体坐标系→世界坐标系
  // ===== 传感器回调函数 =====
  void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);  // 标准点云回调 (Velodyne/Ouster等)
  void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in);  // Livox自定义消息回调
  void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in);  // IMU数据回调
  void img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in);  // 图像数据回调
  // ===== 发布函数 =====
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);  // 发布跟踪图像
  void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager);  // 发布世界坐标系点云
  void publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap);  // 发布视觉子地图
  void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);  // 发布有效匹配点
  void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pmavros_pose_publisherubOdomAftMapped);  // 发布里程计
  void publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher);  // 发布MAVROS位姿
  void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath);  // 发布路径轨迹
  void readParameters(rclcpp::Node::SharedPtr &node);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

  // ===== 线程安全机制 =====
  std::mutex mtx_buffer, mtx_buffer_imu_prop;  // 互斥锁：保护传感器缓冲区和IMU传播
  std::condition_variable sig_buffer;  // 条件变量：通知主循环有新数据

  // ===== 系统配置 =====
  SLAM_MODE slam_mode_;  // 当前SLAM工作模式 (LIVO/LIO/LO)
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;  // 全局体素八叉树地图
  
  string root_dir;  // 项目根目录
  string lid_topic, imu_topic, seq_name, img_topic, world_frame = "camera_init";  // ROS话题名称
  V3D extT;  // IMU到LiDAR外参平移
  M3D extR;  // IMU到LiDAR外参旋转

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  // ===== 噪声参数 =====
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;  // 陀螺仪/加计/逆曝光时间的过程噪声
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, img_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int img_save_interval = 1, pcd_save_interval = -1, pcd_save_type = 0;
  int pub_scan_num = 1;

  // ===== 状态传播 =====
  StatesGroup imu_propagate, latest_ekf_state;  // IMU传播状态和最新EKF状态

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::msg::Imu> prop_imu_buffer;
  sensor_msgs::msg::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::msg::Odometry imu_prop_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  bool navigation_cloud_on_lio = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  int outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  // ===== 传感器数据缓冲区 (双端队列) =====
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;  // 原始LiDAR点云缓冲
  deque<double> lid_header_time_buffer;  // LiDAR时间戳缓冲
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;  // IMU消息缓冲
  deque<cv::Mat> img_buffer;  // 图像缓冲
  deque<double> img_time_buffer;  // 图像时间戳缓冲
  vector<pointWithVar> _pv_list;  // 带协方差的点列表（用于LIO残差计算）
  // ===== 外参配置 =====
  vector<double> extrinT;  // LiDAR-IMU外参平移 (从参数服务器读取)
  vector<double> extrinR;  // LiDAR-IMU外参旋转
  vector<double> cameraextrinT;  // LiDAR-Camera外参平移
  vector<double> cameraextrinR;  // LiDAR-Camera外参旋转
  int IMG_POINT_COV;  // 图像点观测协方差

  // ===== 点云缓冲区 =====
  PointCloudXYZI::Ptr visual_sub_map;  // 视觉子地图（用于可视化）
  PointCloudXYZI::Ptr feats_undistort;  // 去畸变后的特征点
  PointCloudXYZI::Ptr feats_down_body;  // 下采样后的体坐标系点
  PointCloudXYZI::Ptr feats_down_world;  // 下采样后的世界坐标系点
  PointCloudXYZI::Ptr pcl_w_wait_pub;  // 等待发布的点云
  PointCloudXYZI::Ptr pcl_wait_pub;  // 待发布点云
  PointCloudXYZRGB::Ptr pcl_wait_save;  // 待保存的RGB点云
  PointCloudXYZI::Ptr pcl_wait_save_intensity;  // 待保存的强度点云

  ofstream fout_pre, fout_out, fout_visual_pos, fout_lidar_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  // ===== LiDAR测量组 =====
  LidarMeasureGroup LidarMeasures;  // 当前正在处理的LiDAR测量数据
  StatesGroup _state;  // 当前EKF状态
  StatesGroup  state_propagat;  // IMU传播后的先验状态

  nav_msgs::msg::Path path;
  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::Quaternion geoQuat;
  geometry_msgs::msg::PoseStamped msg_body_pose;

  // ===== 核心子模块指针 =====
  PreprocessPtr p_pre;  // LiDAR预处理器（处理多品牌LiDAR数据）
  ImuProcessPtr p_imu;  // IMU处理器（初始化、前向传播、去畸变）
  VoxelMapManagerPtr voxelmap_manager;  // 体素地图管理器（LIO点到平面匹配）
  VIOManagerPtr vio_manager;  // 视觉惯性里程计管理器（直接图像对齐）

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_pub;
  std::shared_ptr<rclcpp::SubscriptionBase> sub_pcl;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubNormal;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSubVisualMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDyn;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynRmed;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pose_publisher;
  rclcpp::TimerBase::SharedPtr imu_prop_timer;
  rclcpp::Node::SharedPtr node;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif
