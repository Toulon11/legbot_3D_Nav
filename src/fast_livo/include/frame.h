/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

/**
 * @file frame.h
 * @brief 图像帧类，存储图像、位姿和特征观测
 * 
 * 在VIO流程中，每帧图像被包装为Frame对象，包含：
 * - 相机模型、图像金字塔
 * - EKF优化后的位姿 (T_f_w_) 和 IMU先验位姿 (T_f_w_prior_)
 * - 多坐标系转换工具函数 (w2c, w2f, c2f, f2w, f2c)
 */
#ifndef LIVO_FRAME_H_
#define LIVO_FRAME_H_

#include <boost/noncopyable.hpp>
#include <vikit/abstract_camera.h>

class VisualPoint;
struct Feature;

typedef list<Feature *> Features;  // 特征列表 (双向链表)
typedef vector<cv::Mat> ImgPyr;  // 图像金字塔类型

/// A frame saves the image, the associated features and the estimated pose.
/// A frame saves the image, the associated features and the estimated pose.
/// 帧类：保存图像、关联特征和估计位姿
class Frame : boost::noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //!< 静态帧计数器，用于自动分配唯一ID
  static int frame_counter_; //!< Counts the number of created frames. Used to set the unique id.
  //!< 帧唯一ID
  int id_;                   //!< Unique id of the frame.
  //!< 相机模型 (针孔/鱼眼等)
  vk::AbstractCamera *cam_;  //!< Camera model.
  //!< 优化后的位姿: 世界坐标系 → 相机坐标系
  SE3<double> T_f_w_;                //!< Transform (f)rame from (w)orld.
  //!< IMU先验位姿: 世界坐标系 → 相机坐标系
  SE3<double> T_f_w_prior_;          //!< Transform (f)rame from (w)orld provided by the IMU prior.
  //!< 当前帧图像
  cv::Mat img_;              //!< Image of the frame.
  //!< 本帧观测到的特征列表
  Features fts_;             //!< List of features in the image.

  Frame(vk::AbstractCamera *cam, const cv::Mat &img);  // 构造并自动创建图像金字塔
  ~Frame();

  /// Initialize new frame and create image pyramid.
  void initFrame(const cv::Mat &img);  // 初始化帧并创建图像金字塔

  /// Return number of point observations.
  /// 返回特征观测数量
  inline size_t nObs() const { return fts_.size(); }

  /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c).
  /// 世界坐标 → 像素坐标 (使用优化后位姿)
  inline Vector2d w2c(const Vector3d &xyz_w) const { return cam_->world2cam(T_f_w_ * xyz_w); }

  /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c) using the IMU prior pose.
  /// 世界坐标 → 像素坐标 (使用IMU先验位姿)
  inline Vector2d w2c_prior(const Vector3d &xyz_w) const { return cam_->world2cam(T_f_w_prior_ * xyz_w); }
  
  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  /// 像素坐标 → 单位球坐标 (归一化射线)
  inline Vector3d c2f(const Vector2d &px) const { return cam_->cam2world(px[0], px[1]); }

  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  inline Vector3d c2f(const double x, const double y) const { return cam_->cam2world(x, y); }

  /// Transforms point coordinates in world-frame (w) to camera-frams (f).
  /// 世界坐标 → 相机坐标
  inline Vector3d w2f(const Vector3d &xyz_w) const { return T_f_w_ * xyz_w; }

  /// Transforms point from frame unit sphere (f) frame to world coordinate frame (w).
  /// 相机坐标 → 世界坐标
  inline Vector3d f2w(const Vector3d &f) const { return T_f_w_.inverse() * f; }

  /// Projects Point from unit sphere (f) in camera pixels (c).
  /// 相机坐标 → 像素坐标
  inline Vector2d f2c(const Vector3d &f) const { return cam_->world2cam(f); }

  /// Return the pose of the frame in the (w)orld coordinate frame.
  /// 获取相机在世界坐标系中的位置
  inline Vector3d pos() const { return T_f_w_.inverse().translation(); }
};

typedef std::unique_ptr<Frame> FramePtr;  // 帧的智能指针

/// Some helper functions for the frame object.
/// 帧工具函数命名空间
namespace frame_utils
{

/// Creates an image pyramid of half-sampled images.
/// 创建半采样图像金字塔 (用于多尺度视觉跟踪)
void createImgPyramid(const cv::Mat &img_level_0, int n_levels, ImgPyr &pyr);

} // namespace frame_utils

#endif // LIVO_FRAME_H_
