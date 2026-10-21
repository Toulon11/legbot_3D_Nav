/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

/**
 * @file voxel_map.h
 * @brief 体素八叉树地图系统，同时服务于 LIO 和 VIO
 * 
 * 核心设计：
 * 1. 全局地图由哈希体素 (VOXEL_LOCATION) 索引
 * 2. 每个体素内部使用八叉树 (VoxelOctoTree) 进行层次化平面拟合
 * 3. 平面拟合通过协方差矩阵特征分解实现，用6x6雅可比传播平面参数不确定性
 * 4. LIO 使用点到平面残差进行 EKF 更新
 * 5. 支持地图滑动窗口 (mapSliding) 限制内存使用
 */
#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include "common_lib.h"
#include <Eigen/Dense>
#include <fstream>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#define VOXELMAP_HASH_P 116101  // 体素哈希的大质数 (用于减少哈希冲突)
#define VOXELMAP_MAX_N 10000000000  // 哈希值上界

static int voxel_plane_id = 0;

/** 体素地图配置参数 */
typedef struct VoxelMapConfig
{
  double max_voxel_size_;  // 最大体素尺寸
  int max_layer_;  // 八叉树最大层数
  int max_iterations_;  // EKF最大迭代次数
  std::vector<int64_t> layer_init_num_;  // 每层初始化所需最少点数
  int max_points_num_;  // 单个节点最大点数
  double planner_threshold_;  // 平面度阈值
  double beam_err_;  // 激光束测距误差
  double dept_err_;  // 深度误差
  double sigma_num_;  // 马氏距离门控阈值 (sigma倍数)
  bool is_pub_plane_map_;  // 是否发布平面地图

  // config of local map sliding
  // 地图滑动窗口配置
  double sliding_thresh;  // 触发滑动的距离阈值
  bool map_sliding_en;  // 是否启用地图滑动
  int half_map_size;  // 滑动窗口半尺寸 (体素数)
} VoxelMapConfig;

/** 点到平面残差结构，用于LIO的EKF更新 */
typedef struct PointToPlane
{
  Eigen::Vector3d point_b_;  // 雷达点在雷达坐标系下的坐标
  Eigen::Vector3d point_w_;  // 雷达点在世界坐标系下的坐标
  Eigen::Vector3d normal_;  // 雷达点匹配平面的法向量（世界系）
  Eigen::Vector3d center_;  // 雷达点匹配的平面中心点坐标（世界系）
  Eigen::Matrix<double, 6, 6> plane_var_;  // 平面参数不确定性 (6x6)
  M3D body_cov_;  // 体坐标系下的点测量协方差
  int layer_;  // 匹配的八叉树层数
  double d_;  // 点到平面距离
  double eigen_value_;  // 最小特征值 (平面度指标)
  bool is_valid_;  // 残差是否有效
  float dis_to_plane_;  // 到平面的距离
} PointToPlane;

/** 体素平面参数，存储拟合结果和不确定性 */
typedef struct VoxelPlane
{
  Eigen::Vector3d center_;  // 平面中心 (点云质心)
  Eigen::Vector3d normal_;  // 平面法向量 (最小特征值对应的特征向量)
  Eigen::Vector3d y_normal_;  // 次小特征向量
  Eigen::Vector3d x_normal_;  // 最大特征向量
  Eigen::Matrix3d covariance_;  // 点云协方差矩阵
  Eigen::Matrix<double, 6, 6> plane_var_;  // 平面参数不确定性 (center+normal 的联合协方差)
  float radius_ = 0;  // 平面拟合半径
  float min_eigen_value_ = 1;  // 最小特征值 (越小越平面)
  float mid_eigen_value_ = 1;  // 中间特征值
  float max_eigen_value_ = 1;  // 最大特征值
  float d_ = 0;  // 平面方程中的 d = -normal·center
  int points_size_ = 0;  // 平面上的点数
  bool is_plane_ = false;  // 是否已形成有效平面
  bool is_init_ = false;  // 是否已初始化
  int id_ = 0;  // 平面唯一ID
  bool is_update_ = false;  // 是否被增量更新过
  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

/**
 * @brief 体素位置索引，用于哈希表快速查找
 * 
 * 哈希函数使用质数乘法散列，减少体素坐标的哈希冲突
 */
class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// Hash value
namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
} // namespace std

struct DS_POINT
{
  float xyz[3];
  float intensity;
  int count = 0;
};

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

/**
 * @brief 体素八叉树，每个体素内部的层次化平面拟合结构
 * 
 * 工作原理：
 * 1. 初始化时收集点数，超过阈值后执行 init_octo_tree()
 * 2. 若点云非平面 (特征值比不满足)，则递归切分为8个子节点 (cut_octo_tree)
 * 3. 每个叶节点维护一个 VoxelPlane
 * 4. 新增点时通过 UpdateOctoTree 增量更新平面参数
 * 5. 查询时通过 find_correspond 遍历八叉树找到最佳匹配平面
 */
class VoxelOctoTree
{

public:
  VoxelOctoTree() = default;
  std::vector<pointWithVar> temp_points_;  // 当前节点累积的点
  VoxelPlane *plane_ptr_;  // 平面参数指针
  int layer_;  // 当前层数 (0=根)
  // 0=叶节点, 1=已切分
  int octo_state_; // 0 is end of tree, 1 is not
  VoxelOctoTree *leaves_[8];  // 8个子节点指针
  // 体素中心坐标
  double voxel_center_[3]; // x, y, z
  std::vector<int> layer_init_num_;  // 每层初始化点数阈值
  float quater_length_;  // 子节点边长的四分之一
  float planer_threshold_;  // 平面度判断阈值
  int points_size_threshold_;  // 触发拟合的点数阈值
  int update_size_threshold_;  // 触发增量更新的点数阈值
  int max_points_num_;  // 单节点最大点数
  int max_layer_;  // 八叉树最大层数
  int new_points_;  // 新增点计数
  bool init_octo_;  // 八叉树是否已初始化
  bool update_enable_;  // 是否允许增量更新

  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planer_threshold_(planer_threshold)
  {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    for (int i = 0; i < 8; i++)
    {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new VoxelPlane;
  }

  ~VoxelOctoTree()
  {
    for (int i = 0; i < 8; i++)
    {
      delete leaves_[i];
    }
    delete plane_ptr_;
  }
  void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);  // 协方差特征分解拟合平面
  void init_octo_tree();  // 初始化八叉树（判断平面度，必要时切分）
  void cut_octo_tree();  // 递归切分八叉树
  void UpdateOctoTree(const pointWithVar &pv);  // 增量更新八叉树

  VoxelOctoTree *find_correspond(Eigen::Vector3d pw);  // 查找最佳匹配平面
  VoxelOctoTree *Insert(const pointWithVar &pv);  // 插入新点
};

void loadVoxelConfig(rclcpp::Node::SharedPtr &node, VoxelMapConfig &voxel_config);

/**
 * @brief 体素地图管理器，维护全局体素地图并执行LIO状态估计
 * 
 * 主要功能：
 * - BuildVoxelMap(): 初始化全局体素地图
 * - UpdateVoxelMap(): 增量更新体素地图
 * - StateEstimation(): 迭代EKF，用点到平面残差更新状态
 * - BuildResidualListOMP(): OpenMP并行计算残差
 * - mapSliding(): 地图滑动窗口，移除远处体素以节省内存
 */
class VoxelMapManager
{
public:
  VoxelMapManager() = default;
  VoxelMapConfig config_setting_;  // 地图配置参数
  int current_frame_id_ = 0;  // 当前帧ID
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_map_pub_;  // 体素地图发布器
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_;  // 全局体素地图

  PointCloudXYZI::Ptr feats_undistort_;  //去畸变后的雷达点云 (所有雷达点全部投射到最后一个imu采样"时刻对应"的雷达系中)
  PointCloudXYZI::Ptr feats_down_body_;  //下采样后的雷达点云（雷达系）
  PointCloudXYZI::Ptr feats_down_world_;  //下采样后的雷达点云（世界系）

  M3D extR_;  // LiDAR到IMU外参旋转
  V3D extT_;  // LiDAR到IMU外参平移
  float build_residual_time, ekf_time;  // 计时器
  float ave_build_residual_time = 0.0;  // 平均残差构建时间
  float ave_ekf_time = 0.0;  // 平均EKF时间
  int scan_count = 0;  // 已处理扫描数
  StatesGroup state_;  // 第k次迭代的滤波后的状态
  V3D position_last_;  // 上一帧位置

  V3D last_slide_position = {0,0,0};  // 上次地图滑动时的位置

  geometry_msgs::msg::Quaternion geoQuat_;

  int feats_down_size_;  // 下采样后的点数
  int effct_feat_num_;  // 下采样之后 还能成功投影到平面上的点数 称为有效特征点数
  std::vector<M3D> cross_mat_list_;  // imu系中雷达点的反对称矩阵列表
  std::vector<M3D> body_cov_list_;  // 雷达点在雷达系下的雷达测量协方差列表
  std::vector<pointWithVar> pv_list_;  // 带协方差的雷达点列表
  std::vector<PointToPlane> ptpl_list_;  // 带有点到平面残差雷达点列表

  VoxelMapManager(VoxelMapConfig &config_setting, std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
      : config_setting_(config_setting), voxel_map_(voxel_map)
  {
    current_frame_id_ = 0;
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
  };

  void StateEstimation(StatesGroup &state_propagat);  // 迭代EKF状态估计（核心函数）
  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);  // LiDAR点云坐标变换

  void BuildVoxelMap();  // 初始化构建体素地图
  V3F RGBFromVoxel(const V3D &input_point);  // 根据体素获取颜色 (可视化用)

  void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);  // 增量更新体素地图

  void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);  // OpenMP并行构建残差

  void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob,
                             PointToPlane &single_ptpl);  // 构建单个点到平面残差（递归遍历八叉树）

  void pubVoxelMap();  // 发布体素地图可视化

  void mapSliding();  // 地图滑动窗口
  void clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min );  // 清除窗口外体素

private:
  void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

  void pubSinglePlane(visualization_msgs::msg::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha,
                      const Eigen::Vector3d rgb);
  void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::msg::Quaternion &q);

  void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif // VOXEL_MAP_H_
