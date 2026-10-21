/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

/**
 * @file vio.h
 * @brief 视觉惯性里程计管理器，实现基于直接图像对齐的视觉跟踪
 * 
 * 核心算法思路：
 * 1. 将图像划分为网格 (grid)，每个网格单元维护若干跟踪点
 * 2. 从体素地图中检索可视化的视觉点 (VisualPoint)
 * 3. 利用仿射变换将参考帧 patch 变换到当前帧，计算光度误差
 * 4. 构建 EKF 雅可比矩阵，用光度残差更新状态
 * 
 * 关键数据结构：
 * - SubSparseMap: 当前帧跟踪的子地图（包含patch、误差、搜索层级等）
 * - VOXEL_POINTS: 体素单元内的视觉点集合
 * - Warp: 缓存的仿射变换矩阵
 */
#ifndef VIO_H_
#define VIO_H_

#include "voxel_map.h"
#include "feature.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/filters/voxel_grid.h>
#include <set>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>

/**
 * @brief 子稀疏地图，存储当前帧跟踪的所有视觉点信息
 * 
 * 每个跟踪点对应：仿射变换后的patch、光度误差、搜索层级、对应的VisualPoint指针
 */
struct SubSparseMap
{
  vector<float> propa_errors;  // 传播误差 (用于鲁棒核函数)
  vector<float> errors;  // 光度误差 (patch差异)
  vector<vector<float>> warp_patch;  // 仿射变换后的图像patch
  vector<int> search_levels;  // 每个点的搜索金字塔层级
  vector<VisualPoint *> voxel_points;  // 对应的视觉地图点指针
  vector<double> inv_expo_list;  // 逆曝光时间列表
  vector<pointWithVar> add_from_voxel_map;  // 从体素地图新增的点

  SubSparseMap()
  {
    propa_errors.reserve(SIZE_LARGE);
    errors.reserve(SIZE_LARGE);
    warp_patch.reserve(SIZE_LARGE);
    search_levels.reserve(SIZE_LARGE);
    voxel_points.reserve(SIZE_LARGE);
    inv_expo_list.reserve(SIZE_LARGE);
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  void reset()
  {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
  }
};

/** 仿射变换缓存，存储每个点的仿射矩阵和搜索层级 */
class Warp
{
public:
  Matrix2d A_cur_ref;  // 参考帧到当前帧的仿射变换矩阵
  int search_level;  // 最佳搜索金字塔层级
  Warp(int level, Matrix2d warp_matrix) : search_level(level), A_cur_ref(warp_matrix) {}
  ~Warp() {}
};

/** 体素单元内的视觉点集合，用于视觉体素地图的快速检索 */
class VOXEL_POINTS
{
public:
  std::vector<VisualPoint *> voxel_points;  // 该体素内的所有视觉点
  int count;  // 点计数
  VOXEL_POINTS(int num) : count(num) {}
  ~VOXEL_POINTS() 
  { 
    for (VisualPoint* vp : voxel_points) 
    {
      if (vp != nullptr) { delete vp; vp = nullptr; }
    }
  }
};

/**
 * @brief 视觉惯性里程计管理器
 * 
 * 主要流程 (processFrame):
 * 1. resetGrid()             重置网格状态
 * 2. retrieveFromVisualSparseMap()  从视觉体素地图检索可跟踪点
 * 3. computeJacobianAndUpdateEKF()   计算雅可比并执行EKF更新
 * 4. generateVisualMapPoints()       在空白网格生成新的视觉地图点
 * 5. updateVisualMapPoints()         更新视觉地图点的观测
 * 6. updateReferencePatch()          更新参考patch（基于NCC评分）
 * 
 * 支持两种更新策略：
 * - 前向组合式 (updateState): 在当前帧计算雅可比
 * - 反向组合式 (updateStateInverse): 在参考帧预计算雅可比，更高效
 */
class VIOManager
{
public:
  int grid_size;  // 网格单元尺寸 (像素)
  vk::AbstractCamera *cam;  // 相机模型 (支持针孔/鱼眼等)
  vk::PinholeCamera *pinhole_cam;  // 针孔相机模型 (colmap输出用)
  StatesGroup *state;  // 指向当前EKF状态的指针
  StatesGroup *state_propagat;  // 指向IMU传播状态的指针
  // ===== 外参矩阵链: IMU→LiDAR→Camera =====
  M3D Rli, Rci, Rcl, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;  // 旋转矩阵和投影雅可比
  V3D Pli, Pci, Pcl, Pcw;  // 平移向量
  // ===== 网格状态数组 (每个网格单元一个元素) =====
  vector<int> grid_num;  // 网格内点数
  vector<int> map_index;  // 网格对应的地图索引
  vector<int> border_flag;  // 边界网格标志 (用于射线采样)
  vector<int> update_flag;  // 更新标志
  vector<float> map_dist;  // 网格内最近点距离
  vector<float> scan_value;  // Shi-Tomasi角点响应值
  vector<float> patch_buffer;
  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en, has_ref_patch_cache;
  bool ncc_en = false, colmap_output_en = false;

  int width, height, grid_n_width, grid_n_height, length;
  double image_resize_factor;
  double fx, fy, cx, cy;
  // ===== patch 参数 =====
  int patch_pyrimid_level, patch_size, patch_size_total, patch_size_half, border, warp_len;
  int max_iterations, total_points;

  double img_point_cov, outlier_threshold, ncc_thre;
  
  SubSparseMap *visual_submap;
  std::vector<std::vector<V3D>> rays_with_sample_points;

  double compute_jacobian_time, update_ekf_time;
  double ave_total = 0;
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  int frame_count = 0;
  bool plot_flag;

  Eigen::Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
  Eigen::MatrixXd K, H_sub_inv;

  ofstream fout_camera, fout_colmap;
  // ===== 视觉体素地图 (与LIO体素地图独立) =====
  unordered_map<VOXEL_LOCATION, VOXEL_POINTS *> feat_map;  // 全局视觉体素地图
  unordered_map<VOXEL_LOCATION, int> sub_feat_map;   // 当前帧的子地图索引
  unordered_map<int, Warp *> warp_map;  // 仿射变换缓存 (key=点ID)
  vector<VisualPoint *> retrieve_voxel_points;  // 从体素地图检索到的点
  vector<pointWithVar> append_voxel_points;  // 待追加的新点
  FramePtr new_frame_;  // 当前帧对象
  cv::Mat img_cp, img_rgb, img_test;  // 图像缓存

  enum CellType
  {
    TYPE_MAP = 1,  // 已有地图点覆盖的网格
    TYPE_POINTCLOUD,  // 仅有LiDAR点云投影的网格
    TYPE_UNKNOWN  // 未覆盖的网格
  };

  VIOManager();
  ~VIOManager();
  void updateStateInverse(cv::Mat img, int level);
  void updateState(cv::Mat img, int level);
  void processFrame(cv::Mat &img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time);
  void retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg);
  void setImuToLidarExtrinsic(const V3D &transl, const M3D &rot);
  void setLidarToCameraExtrinsic(vector<double> &R, vector<double> &P);
  void initializeVIO();
  void getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level);
  void computeProjectionJacobian(V3D p, MD(2, 3) & J);
  void computeJacobianAndUpdateEKF(cv::Mat img);
  void resetGrid();
  void updateVisualMapPoints(cv::Mat img);
  void getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref, const SE3<double> &T_cur_ref,
                           const int level_ref, 
                           const int pyramid_level, const int halfpatch_size, Matrix2d &A_cur_ref);
  void getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const V2D &px_ref,
                                     const V3D &xyz_ref, const V3D &normal_ref, const SE3<double> &T_cur_ref, const int level_ref, Matrix2d &A_cur_ref);
  void warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level,
                  const int pyramid_level, const int halfpatch_size, float *patch);
  void insertPointIntoVoxelMap(VisualPoint *pt_new);
  void plotTrackedPoints();
  void updateFrameState(StatesGroup state);
  void projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void precomputeReferencePatches(int level);
  void dumpDataForColmap();
  double calculateNCC(float *ref_patch, float *cur_patch, int patch_size);
  int getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level);
  V3F getInterpolatedPixel(cv::Mat img, V2D pc);
  
  // void resetRvizDisplay();
  // deque<VisualPoint *> map_cur_frame;
  // deque<VisualPoint *> sub_map_ray;
  // deque<VisualPoint *> sub_map_ray_fov;
  // deque<VisualPoint *> visual_sub_map_cur;
  // deque<VisualPoint *> visual_converged_point;
  // std::vector<std::vector<V3D>> sample_points;

  // PointCloudXYZI::Ptr pg_down;
  // pcl::VoxelGrid<PointType> downSizeFilter;
};
typedef std::shared_ptr<VIOManager> VIOManagerPtr;

#endif // VIO_H_
