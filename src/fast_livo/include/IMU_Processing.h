/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

/**
 * @file IMU_Processing.h
 * @brief IMU处理模块：初始化、前向传播、反向去畸变
 * 
 * 核心功能：
 * 1. IMU_init(): 通过平均前N帧IMU数据估计重力方向和初始偏置
 * 2. UndistortPcl(): 前向传播IMU状态+协方差，然后反向去畸变LiDAR点云
 * 3. Forward_without_imu(): 无IMU时的简化传播 (匀速模型)
 * 4. Process2(): 根据状态路由到上述三种处理模式
 * 
 * 协方差传播原理：
 * - F_x 矩阵包含旋转、位置、速度、偏置的线性化状态转移
 * - cov_w 为过程噪声协方差
 * - P = F*P*F^T + cov_w 的离散化传播
 */
#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <Eigen/Eigen>
#include <fstream>
#include "common_lib.h"
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <utils/so3_math.h>
#include <fstream>
extern const bool time_list(PointType &x, PointType &y);  // 按时间戳排序点

/// *************IMU Process and undistortion
/**
 * @brief IMU处理类，实现IMU驱动的LiDAR点云去畸变和状态传播
 */
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);  // 设置LiDAR-IMU外参 (旋转+平移)
  void set_extrinsic(const V3D &transl);  // 设置LiDAR-IMU外参 (仅平移)
  void set_extrinsic(const MD(4, 4) & T);  // 设置LiDAR-IMU外参 (4x4变换矩阵)
  void set_gyr_cov_scale(const V3D &scaler);  // 设置陀螺仪噪声标准差
  void set_acc_cov_scale(const V3D &scaler);  // 设置加计噪声标准差
  void set_gyr_bias_cov(const V3D &b_g);  // 设置陀螺仪偏置随机游走协方差
  void set_acc_bias_cov(const V3D &b_a);  // 设置加计偏置随机游走协方差
  void set_inv_expo_cov(const double &inv_expo);  // 设置逆曝光时间协方差
  void set_imu_init_frame_num(const int &num);  // 设置IMU初始化所需帧数
  void set_init_acc_norm_range(double min_norm, double max_norm);  // 初始化时允许的平均加速度模长
  void disable_imu();  // 禁用IMU (切换为匀速模型)
  void disable_gravity_est();  // 禁用重力估计
  void disable_bias_est();  // 禁用偏置估计
  void disable_exposure_est();  // 禁用曝光时间估计
  void Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);  // 主入口：IMU处理
  void UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);  // 点云去畸变

  ofstream fout_imu;
  double IMU_mean_acc_norm;
  V3D unbiased_gyr;

  V3D cov_acc;  // 加计测量噪声协方差
  V3D cov_gyr;  // 陀螺仪测量噪声协方差
  V3D cov_bias_gyr;  // 陀螺仪偏置随机游走协方差
  V3D cov_bias_acc;  // 加计偏置随机游走协方差
  double cov_inv_expo;  // 逆曝光时间过程噪声
  double first_lidar_time;  // 第一帧LiDAR时间戳
  bool imu_time_init = false;  // IMU时间戳是否已初始化
  bool imu_need_init = true;  // 是否还在初始化阶段
  M3D Eye3d;
  V3D Zero3d;
  int lidar_type;

private:
  // ===== 私有方法 =====
  void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);  // IMU初始化：平均加速度估计重力
  void Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);  // 无IMU的简化传播
  PointCloudXYZI pcl_wait_proc;  // 待处理的点云
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu;  // 上一帧IMU消息
  PointCloudXYZI::Ptr cur_pcl_un_;  // 当前去畸变后的点云
  vector<Pose6D> IMUpose;  // IMU传播过程中保存的位姿序列 (用于反向去畸变)
  M3D Lid_rot_to_IMU;  // LiDAR到IMU的旋转外参
  V3D Lid_offset_to_IMU;  // LiDAR到IMU的平移外参
  V3D mean_acc;  // 平均加速度 (用于初始化)
  V3D mean_gyr;  // 平均角速度
  V3D angvel_last;  // 上一时刻角速度
  V3D acc_s_last;  // 上一时刻加速度
  double last_prop_end_time;  // 上次传播结束时间
  double time_last_scan;  // 上次LiDAR扫描时间
  int init_iter_num = 1, MAX_INI_COUNT = 20;  // 初始化参数
  double init_acc_min_norm = 1e-6, init_acc_max_norm = 1e9;
  bool b_first_frame = true;  // 是否第一帧
  bool imu_en = true;  // 是否启用IMU
  bool gravity_est_en = true;  // 是否估计重力
  bool ba_bg_est_en = true;  // 是否估计偏置
  bool exposure_estimate_en = true;  // 是否估计曝光时间
};
typedef std::shared_ptr<ImuProcess> ImuProcessPtr;
#endif
