/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

/**
 * @file visual_point.h
 * @brief 视觉地图点，存储3D位置、表面法线、观测历史和参考patch
 * 
 * 每个VisualPoint代表场景表面上的一个点，包含：
 * - 3D世界坐标 + 协方差
 * - 表面法线 (用于仿射变换和参考patch选择)
 * - 多帧观测列表 (obs_)，每帧观测为一个Feature
 * - 参考patch (ref_patch)，用于直接图像对齐的参考模板
 */
#ifndef LIVO_POINT_H_
#define LIVO_POINT_H_

#include <boost/noncopyable.hpp>
#include "common_lib.h"
#include "frame.h"

class Feature;

/// A visual map point on the surface of the scene.
/// 视觉地图点：场景表面的3D点
class VisualPoint : boost::noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //!< 世界坐标系中的三维坐标
  Vector3d pos_;                //!< 3d pos of the point in the world coordinate frame.
  //!< 表面法向量 (用于仿射变换)
  Vector3d normal_;             //!< Surface normal at point.
  //!< 法向量估计的信息矩阵 (逆协方差)
  Matrix3d normal_information_; //!< Inverse covariance matrix of normal estimation.
  //!< 上次更新的法向量 (用于收敛判断)
  Vector3d previous_normal_;    //!< Last updated normal vector.
  //!< 观测该点的特征列表 (多帧观测)
  list<Feature *> obs_;         //!< Reference patches which observe the point.
  //!< 位置协方差
  Eigen::Matrix3d covariance_;  //!< Covariance of the point.
  //!< 点是否已收敛 (位置稳定)
  bool is_converged_;           //!< True if the point is converged.
  //!< 法向量是否已初始化
  bool is_normal_initialized_;  //!< True if the normal is initialized.
  //!< 是否已设置参考patch
  bool has_ref_patch_;          //!< True if the point has a reference patch.
  //!< 参考patch (用于直接法图像对齐)
  Feature *ref_patch;           //!< Reference patch of the point.

  VisualPoint(const Vector3d &pos);  // 构造函数
  ~VisualPoint();  // 析构：释放所有观测特征
  void findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const;  // 查找NCC评分最低的特征
  void deleteNonRefPatchFeatures();  // 删除非参考patch的所有特征
  void deleteFeatureRef(Feature *ftr);  // 删除指定特征引用
  void addFrameRef(Feature *ftr);  // 添加新的帧观测
  bool getCloseViewObs(const Vector3d &pos, Feature *&obs, const Vector2d &cur_px) const;  // 获取视角最接近的观测 (>60°拒绝)
};

#endif // LIVO_POINT_H_
