/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

/**
 * @file common_lib.h
 * @brief FAST-LIVO2 公共库头文件，定义核心数据结构和类型
 * 
 * 包含以下核心内容：
 * 1. StatesGroup —— 19维EKF状态向量（旋转、位置、逆曝光时间、速度、陀螺仪/加计偏置、重力）
 * 2. pointWithVar —— 带协方差的三维点，支持多坐标系转换
 * 3. LidarMeasureGroup —— 多传感器时间同步容器
 * 4. 枚举类型 —— LiDAR类型、SLAM模式、EKF状态
 * 
 * 状态向量布局 (19维):
 *   [0-2]   rot_end       旋转矩阵的李代数表示 (SO(3))
 *   [3-5]   pos_end       位置 (世界坐标系)
 *   [6]     inv_expo_time 逆曝光时间 (用于自动曝光补偿)
 *   [7-9]   vel_end       速度 (世界坐标系)
 *   [10-12] bias_g        陀螺仪偏置
 *   [13-15] bias_a        加计偏置
 *   [16-18] gravity       重力加速度估计
 *
 */
#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <utils/so3_math.h>
#include <utils/types.h>
#include <utils/color.h>
#include <utils/utils.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sophus/se3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

using namespace std;
// using namespace Eigen;   // avoid cmake error: reference to ‘Matrix’ is ambiguous
using namespace Sophus;

#define print_line std::cout << __FILE__ << ", " << __LINE__ << std::endl;  // 调试用：打印文件名和行号
// 重力加速度常量 (m/s^2)
#define G_m_s2 (9.81)   // Gravaty const in GuangDong/China
// EKF状态维度 (SO(3)视为3维)
#define DIM_STATE (19)  // Dimension of states (Let Dim(SO(3)) = 3)
#define INIT_COV (0.01)  // 初始协方差值
#define SIZE_LARGE (500)  // 容器大预分配尺寸 (用于视觉子地图)
#define SIZE_SMALL (100)  // 容器小预分配尺寸
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]  // 从数组构造 Eigen::Vector3d
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]  // 从数组构造 Eigen::Matrix3d
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))  // 调试文件路径宏

/** LiDAR 类型枚举，支持多种雷达输入及仿真 XYZI */
enum LID_TYPE
{
  AVIA = 1,  // Livox Avia (Livox 自定义消息格式)
  VELO16 = 2,
  OUST64 = 3,
  L515 = 4,
  XT32 = 5,  // 禾赛 XT32
  PANDAR128 = 6,  // 禾赛 Pandar128
  ROBOSENSE = 7,  // 速腾聚创 Airy
  SIM_XYZI = 8
};
/** SLAM 工作模式枚举 */
enum SLAM_MODE
{
  ONLY_LO = 0,  // 仅 LiDAR 里程计 (无IMU)
  ONLY_LIO = 1,  // LiDAR + IMU 融合里程计
  LIVO = 2  // LiDAR + IMU + 视觉 完整融合里程计
};
/** EKF 处理状态枚举，标识当前帧正在执行的阶段 */
enum EKF_STATE
{
  WAIT = 0,  // 等待数据同步
  VIO = 1,  // 正在执行视觉惯性里程计更新
  LIO = 2,  // 正在执行LiDAR惯性里程计更新
  LO = 3  // 正在执行纯LiDAR里程计更新
};

/**
 * @brief 单帧测量数据组，包含一个VIO时刻的IMU队列和图像
 * 
 * 用于多传感器时间同步：每帧图像对应一组IMU数据
 */
struct MeasureGroup
{
  double vio_time;  // 图像采集时间戳
  double lio_time;  // LiDAR扫描时间戳
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;  // 两帧图像之间的IMU消息队列
  cv::Mat img;  // 当前帧图像
  MeasureGroup()
  {
    vio_time = 0.0;
    lio_time = 0.0;
  };
};

/**
 * @brief LiDAR测量组，管理一帧LiDAR扫描的完整生命周期
 * 
 * 核心功能：
 * - 存储LiDAR帧的起止时间
 * - 通过 EKF_STATE 标志位控制当前处于 LIO 还是 VIO 处理阶段
 * - 支持子扫描分割：在图像采集时刻将LiDAR点云拆分为 pcl_proc_cur 和 pcl_proc_next
 *   这是实现 LIO-VIO 亚扫描级时间同步的关键机制
 */
struct LidarMeasureGroup
{
  double lidar_frame_beg_time;  // LiDAR帧起始时间戳
  double lidar_frame_end_time;  // LiDAR帧结束时间戳
  double last_lio_update_time;  // 上一次LIO状态更新的时间戳
  PointCloudXYZI::Ptr lidar;  // 原始LiDAR点云
  PointCloudXYZI::Ptr pcl_proc_cur;  // 当前处理段的点云（图像时刻之前）
  PointCloudXYZI::Ptr pcl_proc_next;  // 下一处理段的点云（图像时刻之后）
  deque<struct MeasureGroup> measures;  // 一帧LiDAR内包含的多组图像+IMU测量
  EKF_STATE lio_vio_flg;  // 当前EKF处理状态标志
  int lidar_scan_index_now;  // 当前处理的LiDAR扫描索引

  LidarMeasureGroup()
  {
    lidar_frame_beg_time = -0.0;
    lidar_frame_end_time = 0.0;
    last_lio_update_time = -1.0;
    lio_vio_flg = WAIT;
    this->lidar.reset(new PointCloudXYZI());
    this->pcl_proc_cur.reset(new PointCloudXYZI());
    this->pcl_proc_next.reset(new PointCloudXYZI());
    this->measures.clear();
    lidar_scan_index_now = 0;
    last_lio_update_time = -1.0;
  };
};

/**
 * @brief 带协方差的三维点，用于LiDAR点的不确定性传播
 * 
 * 存储同一个点在不同坐标系下的坐标（body/lidar/world），
 * 以及测量协方差（用于点到平面残差的马氏距离计算）
 */
typedef struct pointWithVar
{
  // LiDAR体坐标系下的点坐标
  Eigen::Vector3d point_b;     // point in the lidar body frame
  // IMU体坐标系下的点坐标
  Eigen::Vector3d point_i;     // point in the imu body frame
  // 世界坐标系下的点坐标
  Eigen::Vector3d point_w;     // point in the world frame
  // 去除状态协方差后的纯测量协方差
  Eigen::Matrix3d var_nostate; // the var removed the state covarience
  Eigen::Matrix3d body_var;  // 雷达坐标系下的雷达测量协方差
  Eigen::Matrix3d var;  // 世界坐标系下雷达点的总协方差
  Eigen::Matrix3d point_crossmat;  // 点的反对称矩阵 (用于雅可比计算)
  Eigen::Vector3d normal;  // 点所在平面的法向量
  pointWithVar()
  {
    var_nostate = Eigen::Matrix3d::Zero();
    var = Eigen::Matrix3d::Zero();
    body_var = Eigen::Matrix3d::Zero();
    point_crossmat = Eigen::Matrix3d::Zero();
    point_b = Eigen::Vector3d::Zero();
    point_i = Eigen::Vector3d::Zero();
    point_w = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
  };
} pointWithVar;


/**
 * @brief EKF 状态组，维护19维状态向量及其协方差
 * 
 * 核心设计：通过重载 operator+ / operator- 实现流形上的加减运算
 * - operator+ : 用指数映射 Exp() 将增量向量加到旋转矩阵上 (右乘)
 * - operator- : 用对数映射 Log() 提取两个旋转矩阵之间的差异
 * 这是处理 SO(3) 李群上状态估计的标准方法，避免了四元数归一化问题
 */
struct StatesGroup
{
  StatesGroup()
  {
    this->rot_end = M3D::Identity();
    this->pos_end = V3D::Zero();
    this->vel_end = V3D::Zero();
    this->bias_g = V3D::Zero();
    this->bias_a = V3D::Zero();
    this->gravity = V3D::Zero();
    this->inv_expo_time = 1.0;  // 逆曝光时间初始化为1.0 (无缩放)
    this->cov = MD(DIM_STATE, DIM_STATE)::Identity() * INIT_COV;
    this->cov(6, 6) = 0.00001;  // 逆曝光时间初始不确定性较小
    this->cov.block<9, 9>(10, 10) = MD(9, 9)::Identity() * 0.00001;  // bias和gravity初始不确定性较小
  };

  StatesGroup(const StatesGroup &b)
  {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->inv_expo_time = b.inv_expo_time;
    this->cov = b.cov;
  };

  StatesGroup &operator=(const StatesGroup &b)
  {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->inv_expo_time = b.inv_expo_time;
    this->cov = b.cov;
    return *this;
  };

  // 流形上的加法: state ⊕ delta，旋转部分使用右乘指数映射
  StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add)
  {
    StatesGroup a;
    // 旋转: R_new = R_old * Exp(delta_phi)，其中 delta_phi 是 so(3) 上的增量
    a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    // 欧几里得空间直接相加: 位置、逆曝光时间、速度、偏置、重力
    a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
    a.inv_expo_time = this->inv_expo_time + state_add(6, 0);
    a.vel_end = this->vel_end + state_add.block<3, 1>(7, 0);
    a.bias_g = this->bias_g + state_add.block<3, 1>(10, 0);
    a.bias_a = this->bias_a + state_add.block<3, 1>(13, 0);
    a.gravity = this->gravity + state_add.block<3, 1>(16, 0);

    a.cov = this->cov;
    return a;
  };

  StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
  {
    this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    this->pos_end += state_add.block<3, 1>(3, 0);
    this->inv_expo_time += state_add(6, 0);
    this->vel_end += state_add.block<3, 1>(7, 0);
    this->bias_g += state_add.block<3, 1>(10, 0);
    this->bias_a += state_add.block<3, 1>(13, 0);
    this->gravity += state_add.block<3, 1>(16, 0);
    return *this;
  };

  // 流形上的减法: state1 ⊖ state2，旋转部分使用对数映射提取差异
  Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b)
  {
    Matrix<double, DIM_STATE, 1> a;
    // 旋转差异: Log(R2^T * R1)，得到 so(3) 上的三维向量
    M3D rotd(b.rot_end.transpose() * this->rot_end);
    a.block<3, 1>(0, 0) = Log(rotd);
    a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
    a(6, 0) = this->inv_expo_time - b.inv_expo_time;
    a.block<3, 1>(7, 0) = this->vel_end - b.vel_end;
    a.block<3, 1>(10, 0) = this->bias_g - b.bias_g;
    a.block<3, 1>(13, 0) = this->bias_a - b.bias_a;
    a.block<3, 1>(16, 0) = this->gravity - b.gravity;
    return a;
  };

  void resetpose()
  {
    this->rot_end = M3D::Identity();
    this->pos_end = V3D::Zero();
    this->vel_end = V3D::Zero();
  }

  // ===== 19维状态向量 =====
  // 最后一个imu采样的imu坐标系到世界坐标系的旋转矩阵 扫描结束时的旋转矩阵 [0-2], SO(3)李群
  M3D rot_end;                              // the estimated attitude (rotation matrix) at the end lidar point
  // 最后一个imu采样的imu坐标系在世界坐标系的位置 扫描结束时的位置 [3-5], 世界坐标系
  V3D pos_end;                              // the estimated position at the end lidar point (world frame)
  // 最后一个imu采样的imu坐标系在世界坐标系的速度 扫描结束时的速度 [7-9], 世界坐标系
  V3D vel_end;                              // the estimated velocity at the end lidar point (world frame)
  // 逆曝光时间 [6], 用于光度误差的曝光补偿
  double inv_expo_time;                     // the estimated inverse exposure time (no scale)
  // 陀螺仪偏置 [10-12]
  V3D bias_g;                               // gyroscope bias
  // 加速度计偏置 [13-15]
  V3D bias_a;                               // accelerator bias
  // 重力加速度估计 [16-18]
  V3D gravity;                              // the estimated gravity acceleration
  // 19x19 误差状态协方差矩阵
  Matrix<double, DIM_STATE, DIM_STATE> cov; // states covariance
};

/**
 * @brief 构造 Pose6D 结构体，存储IMU传播中间结果
 * 
 * 在IMU前向传播和反向去畸变过程中，每个IMU时刻的位姿、速度、加速度、角速度
 * 都被保存为 Pose6D，用于后续的反向点云去畸变
 */
template <typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p,
                const Matrix<T, 3, 3> &R)
{
  Pose6D rot_kp;
  rot_kp.offset_time = t;
  for (int i = 0; i < 3; i++)
  {
    rot_kp.acc[i] = a(i);
    rot_kp.gyr[i] = g(i);
    rot_kp.vel[i] = v(i);
    rot_kp.pos[i] = p(i);
    for (int j = 0; j < 3; j++)
      rot_kp.rot[i * 3 + j] = R(i, j);
  }
  // Map<M3D>(rot_kp.rot, 3,3) = R;
  return move(rot_kp);
}

#endif
