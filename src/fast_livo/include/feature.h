/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

/**
 * @file feature.h
 * @brief 图像特征结构，存储某个视觉点在一帧中的观测信息
 * 
 * 每个Feature代表一个VisualPoint在某一帧中的观测：
 * - 图像patch (小块像素数据)
 * - 像素坐标和归一化射线方向
 * - 采集时的位姿、金字塔层级、NCC评分
 * - 逆曝光时间 (用于光度误差归一化)
 */
#ifndef LIVO_FEATURE_H_
#define LIVO_FEATURE_H_

#include "visual_point.h"

// A salient image region that is tracked across frames.
// 图像显著区域特征，在帧间跟踪
struct Feature
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum FeatureType
  {
    CORNER,  // 角点
    EDGELET  // 边缘特征
  };
  int id_;  // 特征唯一ID
  // 特征类型 (角点/边缘)
  FeatureType type_;     //!< Type can be corner or edgelet.
  // 特征所属图像
  cv::Mat img_;          //!< Image associated with the patch feature
  // 像素坐标 (金字塔第0层)
  Vector2d px_;          //!< Coordinates in pixels on pyramid level 0.
  // 归一化射线方向 (单位承载向量)
  Vector3d f_;           //!< Unit-bearing vector of the patch feature.
  // 图像金字塔层级
  int level_;            //!< Image pyramid level where patch feature was extracted.
  // 对应的3D视觉点
  VisualPoint *point_;   //!< Pointer to 3D point which corresponds to the patch feature.
  // 边缘特征的主梯度方向
  Vector2d grad_;        //!< Dominant gradient direction for edglets, normalized.
  // 采集时的相机位姿
  SE3<double> T_f_w_;            //!< Pose of the frame where the patch feature was extracted.
  // 图像patch数据指针 (patch_size x patch_size)
  float *patch_;         //!< Pointer to the image patch data.
  // NCC评分 (跟踪质量指标)
  float score_;          //!< Score of the patch feature.
  // patch平均亮度 (用于归一化)
  float mean_;           //!< Mean intensity of the image patch feature, used for normalization.
  // 采集时的逆曝光时间 (光度补偿)
  double inv_expo_time_; //!< Inverse exposure time of the image where the patch feature was extracted.
  
  // 构造函数：关联3D点、patch、像素坐标、射线方向和位姿
  Feature(VisualPoint *_point, float *_patch, const Vector2d &_px, const Vector3d &_f, const SE3<double> &_T_f_w, int _level)
      : type_(CORNER), px_(_px), f_(_f), T_f_w_(_T_f_w), mean_(0), score_(0), level_(_level), patch_(_patch), point_(_point)
  {
  }

  inline Vector3d pos() const { return T_f_w_.inverse().translation(); }  // 获取采集时的相机位置
  
  ~Feature()
  {
    // ROS_WARN("The feature %d has been destructed.", id_);
    delete[] patch_;  // 释放patch内存
  }
};

#endif // LIVO_FEATURE_H_
