/**
 * @file main.cpp
 * @brief FAST-LIVO2 系统入口文件
 * 
 * FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry
 * 核心思路：通过紧耦合的 LiDAR-惯性-视觉融合实现高精度实时里程计
 * 系统支持三种工作模式：LIVO（完整融合）、ONLY_LIO（仅LiDAR-惯性）、ONLY_LO（仅LiDAR）
 * 
 * 主流程：ROS初始化 → 创建LIVMapper → 初始化订阅/发布 → 进入主循环(run)
 */
#include "LIVMapper.h"

int main(int argc, char **argv)
{
  // 初始化 ROS 2；下方创建的映射节点名称为 "laserMapping"
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr nh;
  // 创建图像传输层，用于高效发布/订阅图像话题
  image_transport::ImageTransport it_(nh);
  // 创建核心映射器对象，构造函数中读取参数并初始化各子模块
  LIVMapper mapper(nh, "laserMapping");
  // 初始化所有 ROS 话题的订阅者和发布者
  mapper.initializeSubscribersAndPublishers(nh, it_);
  // 进入主循环：传感器同步 → IMU传播 → 状态估计与建图
  mapper.run(nh);
  rclcpp::shutdown();
  return 0;
}
