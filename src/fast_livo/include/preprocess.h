/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

/**
 * @file preprocess.h
 * @brief LiDAR数据预处理模块，支持7种不同品牌的LiDAR适配器
 * 
 * 主要功能：
 * 1. 将各品牌LiDAR的ROS消息解析为统一格式 (PointCloudXYZI)
 * 2. 统一输出: x,y,z坐标 + intensity强度 + curvature时间戳
 * 3. 可选的特征提取 (平面/边缘分类)
 * 4. 距离盲区和点云抽取
 * 
 * 支持的LiDAR类型: Avia, Velodyne, Ouster, L515, XT32, Pandar128, Robosense
 */
#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include "common_lib.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)  // NaN/Inf 检测宏

/** LiDAR特征类型枚举 (用于特征提取模式) */
enum LiDARFeature
{
  Nor,  // 普通点
  Poss_Plane,  // 可能的平面点
  Real_Plane,  // 确定的平面点
  Edge_Jump,  // 边缘跳变点
  Edge_Plane,  // 边缘平面点
  Wire,  // 线特征点
  ZeroPoint  // 零点 (无效)
};
/** 邻域方向 */
enum Surround
{
  Prev,  // 前方邻域
  Next  // 后方邻域
};
/** 边缘跳变类型 */
enum E_jump
{
  Nr_nor,  // 正常
  Nr_zero,  // 近零
  Nr_180,  // 近180度
  Nr_inf,  // 无穷远
  Nr_blind  // 盲区内
};

/** 原始点属性，用于特征提取的局部几何分析 */
struct orgtype
{
  double range;  // 点到激光中心的距离平方
  double dista;  // 与相邻点的距离平方
  double angle[2];  // 前后邻域的角度信息
  double intersect;  // 交叉角信息
  E_jump edj[2];  // 前后边缘跳变类型
  LiDARFeature ftype;  // 分类结果
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

/*** Velodyne ***/
/*** Velodyne 自定义点类型 ***/
namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;  // x,y,z 坐标
  float intensity;  // 反射强度
  float time;  // 相对时间偏移 (秒)
  std::uint16_t ring;  // 扫描线编号
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros
// PCL点类型注册宏，使自定义点类型可与PCL框架互操作
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))
/****************/

/*** Ouster ***/
/*** Ouster 自定义点类型 ***/
namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;  // 反射强度
  std::uint32_t t;  // 纳秒级时间戳
  std::uint16_t reflectivity;  // 反射率
  uint8_t ring;  // 扫描线编号
  std::uint16_t ambient;  // 环境光强度
  std::uint32_t range;  // 测量距离
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (std::uint32_t, t, t)(std::uint16_t, reflectivity,
                                                        reflectivity)(std::uint8_t, ring, ring)(std::uint16_t, ambient, ambient)(std::uint32_t, range, range))
/****************/

/*** Hesai_XT32 ***/
/*** 禾赛XT32 自定义点类型 ***/
namespace xt32_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace xt32_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

/*** Hesai_Pandar128 ***/
/*** 禾赛Pandar128 自定义点类型 ***/
namespace Pandar128_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace Pandar128_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(Pandar128_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

/*** Robosense_Airy ***/
/*** 速腾聚创Airy 自定义点类型 ***/
namespace robosense_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace robosense_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

/**
 * @brief LiDAR预处理器类，将各品牌LiDAR原始数据解析为统一格式
 * 
 * 统一输出格式：PointCloudXYZI，其中 curvature 字段存储每个点的时间戳
 * 这是后续 IMU 去畸变和子扫描分割的基础
 */
class Preprocess
{
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

  void process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);  // Livox消息处理
  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);  // 标准点云消息处理
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);  // 设置预处理参数

  // sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud;
  // ===== 点云缓冲区 =====
  PointCloudXYZI pl_full, pl_corn, pl_surf;  // 全部点、角点、面点
  // 按扫描线分组的缓冲区 (最大128线)
  PointCloudXYZI pl_buff[128]; // maximum 128 line lidar
  // 每个扫描线的原始属性
  vector<orgtype> typess[128]; // maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS;  // LiDAR类型、抽取间隔、扫描线数
  
  double blind, blind_sqr;  // 盲区距离及其平方 (滤除近处杂波)
  double scan_period = 0.1;
  bool feature_enabled, given_offset_time;  // 是否启用特征提取、是否提供时间偏移
  // 调试用发布器
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_full;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_surf;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_corn;

private:
  // ===== 各品牌LiDAR专用处理函数 =====
  void avia_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);
  void oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void xt32_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);  // 禾赛 XT32
  void Pandar128_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);  // 禾赛 Pandar128
  void robosense_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);  // 速腾聚创 Airy
  void l515_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void sim_xyzi_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);  // 特征分类 (平面/边缘/线)
  void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);
  int plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  // ===== 特征提取参数 =====
  int group_size;  // 局部分析组大小
  double disA, disB, inf_bound;  // 距离判断阈值
  double limit_maxmid, limit_midmin, limit_maxmin;  // 角度限制阈值
  double p2l_ratio;  // 点到线距离比阈值
  double jump_up_limit, jump_down_limit;  // 边缘跳变上/下限
  double cos160;  // 160度余弦值 (用于判断回折)
  double edgea, edgeb;  // 边缘判断参数
  double smallp_intersect, smallp_ratio;  // 小平面交叉参数
  double vx, vy, vz;  // 相邻点差值缓存
};
typedef std::shared_ptr<Preprocess> PreprocessPtr;

#endif // PREPROCESS_H_
