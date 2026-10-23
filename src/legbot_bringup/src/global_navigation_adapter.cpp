#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace {

using Odometry = nav_msgs::msg::Odometry;
using Cloud = sensor_msgs::msg::PointCloud2;

int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

tf2::Transform pose_transform(const geometry_msgs::msg::Pose &pose)
{
  const auto &p = pose.position;
  const auto &q = pose.orientation;
  return tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w),
                        tf2::Vector3(p.x, p.y, p.z));
}

void write_pose(const tf2::Transform &transform, geometry_msgs::msg::Pose &pose)
{
  const auto &p = transform.getOrigin();
  const auto &q = transform.getRotation();
  pose.position.x = p.x();
  pose.position.y = p.y();
  pose.position.z = p.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
}

void rotate_pose_covariance(const tf2::Transform &transform,
                            std::array<double, 36> &covariance)
{
  tf2::Matrix3x3 rotation(transform.getRotation());
  double jacobian[6][6]{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      jacobian[i][j] = rotation[i][j];
      jacobian[i + 3][j + 3] = rotation[i][j];
    }
  }
  std::array<double, 36> rotated{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      for (int k = 0; k < 6; ++k) {
        for (int l = 0; l < 6; ++l) {
          rotated[i * 6 + j] += jacobian[i][k] * covariance[k * 6 + l] * jacobian[j][l];
        }
      }
    }
  }
  covariance = rotated;
}

}  // namespace

class GlobalNavigationAdapter : public rclcpp::Node
{
public:
  GlobalNavigationAdapter() : Node("global_navigation_adapter")
  {
    source_ = declare_parameter<std::string>("source", "fastlio");
    alignment_mode_ = declare_parameter<std::string>("alignment_mode", "gazebo_truth");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    local_frame_ = declare_parameter<std::string>("local_frame", "odom");
    const std::string prefix = source_ == "fastlivo" ? "/fast_livo" : "/fast_lio";
    const bool gazebo = source_ == "gazebo";
    if (!gazebo && source_ != "fastlio" && source_ != "fastlivo") {
      throw std::invalid_argument("source must be gazebo, fastlio, or fastlivo");
    }
    if (!gazebo && alignment_mode_ != "gazebo_truth" && alignment_mode_ != "spawn_pose") {
      throw std::invalid_argument("alignment_mode must be gazebo_truth or spawn_pose");
    }
    if (global_frame_.empty() || local_frame_.empty() || global_frame_ == local_frame_) {
      throw std::invalid_argument("global_frame and local_frame must be distinct nonempty frames");
    }
    const auto base_topic = declare_parameter<std::string>(
        "base_odom_topic", gazebo ? "/Odometry_gazebo" : prefix + "/odometry_base");
    const auto lidar_topic = declare_parameter<std::string>(
        "lidar_odom_topic", gazebo ? "/go2/lidar_pose" : prefix + "/odometry_lidar");
    const auto cloud_topic = declare_parameter<std::string>(
        "cloud_topic", gazebo ? "/livox/points_world" : prefix + "/cloud_registered");
    const auto truth_topic = declare_parameter<std::string>(
        "truth_odom_topic", "/Odometry_gazebo");
    match_tolerance_ns_ = static_cast<int64_t>(
        declare_parameter<double>("alignment_tolerance", 0.10) * 1e9);
    settle_ns_ = static_cast<int64_t>(
        declare_parameter<double>("alignment_settle_time", 1.0) * 1e9);
    const double spawn_x = declare_parameter<double>("spawn_x", -27.0);
    const double spawn_y = declare_parameter<double>("spawn_y", 6.0);
    const double spawn_z = declare_parameter<double>("spawn_z", 0.5);
    const double spawn_yaw = declare_parameter<double>("spawn_yaw", 0.0);
    if (match_tolerance_ns_ <= 0 || settle_ns_ < 0 ||
        !std::isfinite(spawn_x) || !std::isfinite(spawn_y) ||
        !std::isfinite(spawn_z) || !std::isfinite(spawn_yaw)) {
      throw std::invalid_argument("invalid alignment or spawn parameter");
    }
    tf2::Quaternion spawn_rotation;
    spawn_rotation.setRPY(0.0, 0.0, spawn_yaw);
    spawn_pose_.setOrigin(tf2::Vector3(spawn_x, spawn_y, spawn_z));
    spawn_pose_.setRotation(spawn_rotation);

    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    base_pub_ = create_publisher<Odometry>("/slam/global_odometry_base", output_qos);
    lidar_pub_ = create_publisher<Odometry>("/slam/odometry_lidar", output_qos);
    cloud_pub_ = create_publisher<Cloud>("/slam/global_cloud_registered", output_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/slam/alignment_status", rclcpp::QoS(1).reliable().transient_local());
    if (!gazebo) {
      static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    } else {
      aligned_ = true;
      global_from_local_.setIdentity();
    }

    const auto input_qos = rclcpp::SensorDataQoS();
    base_sub_ = create_subscription<Odometry>(base_topic, input_qos,
        [this](Odometry::ConstSharedPtr msg) { on_base(*msg); });
    lidar_sub_ = create_subscription<Odometry>(lidar_topic, input_qos,
        [this](Odometry::ConstSharedPtr msg) { publish_odom(*msg, lidar_pub_); });
    cloud_sub_ = create_subscription<Cloud>(cloud_topic, input_qos,
        [this](Cloud::ConstSharedPtr msg) { on_cloud(*msg); });
    if (!gazebo && alignment_mode_ == "gazebo_truth") {
      truth_sub_ = create_subscription<Odometry>(truth_topic, input_qos,
          [this](Odometry::ConstSharedPtr msg) { on_truth(*msg); });
    }
    set_status(gazebo ? "aligned: Gazebo world" : "waiting for estimated base odometry");
    RCLCPP_INFO(get_logger(), "source=%s, alignment=%s, base=%s, lidar=%s, cloud=%s",
                source_.c_str(), alignment_mode_.c_str(), base_topic.c_str(),
                lidar_topic.c_str(), cloud_topic.c_str());
  }

private:
  void set_status(const std::string &status)
  {
    if (status == status_) return;
    status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "%s", status.c_str());
  }

  void on_truth(const Odometry &truth)
  {
    if (truth.header.frame_id != local_frame_) return;
    truths_.push_back(truth);
    if (truths_.size() > 200) truths_.pop_front();
    try_align();
  }

  void on_base(const Odometry &base)
  {
    if (base.header.frame_id != local_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Expected base frame %s, got %s", local_frame_.c_str(), base.header.frame_id.c_str());
      return;
    }
    if (!aligned_) {
      pending_base_ = base;
      if (first_estimate_ns_ == 0) first_estimate_ns_ = stamp_ns(base.header.stamp);
      try_align();
    } else {
      publish_odom(base, base_pub_);
    }
  }

  void try_align()
  {
    if (aligned_ || !pending_base_) return;
    const int64_t estimate_ns = stamp_ns(pending_base_->header.stamp);
    if (estimate_ns - first_estimate_ns_ < settle_ns_) return;
    tf2::Transform global_from_base;
    if (alignment_mode_ == "spawn_pose") {
      global_from_base = spawn_pose_;
      set_status("aligning from configured spawn pose (approximate)");
    } else {
      const Odometry *nearest = nullptr;
      int64_t smallest_delta = std::numeric_limits<int64_t>::max();
      for (const auto &truth : truths_) {
        const int64_t delta = std::llabs(stamp_ns(truth.header.stamp) - estimate_ns);
        if (delta < smallest_delta) {
          smallest_delta = delta;
          nearest = &truth;
        }
      }
      if (!nearest || smallest_delta > match_tolerance_ns_) {
        set_status("waiting for timestamp-matched Gazebo ground truth");
        return;
      }
      global_from_base = pose_transform(nearest->pose.pose);
    }
    const auto local_from_base = pose_transform(pending_base_->pose.pose);
    if (local_from_base.getRotation().length2() < 0.25 ||
        global_from_base.getRotation().length2() < 0.25) return;
    global_from_local_ = global_from_base * local_from_base.inverse();
    auto normalized_rotation = global_from_local_.getRotation();
    normalized_rotation.normalize();
    global_from_local_.setRotation(normalized_rotation);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = pending_base_->header.stamp;
    tf.header.frame_id = global_frame_;
    tf.child_frame_id = local_frame_;
    const auto &translation = global_from_local_.getOrigin();
    const auto &rotation = global_from_local_.getRotation();
    tf.transform.translation.x = translation.x();
    tf.transform.translation.y = translation.y();
    tf.transform.translation.z = translation.z();
    tf.transform.rotation.x = rotation.x();
    tf.transform.rotation.y = rotation.y();
    tf.transform.rotation.z = rotation.z();
    tf.transform.rotation.w = rotation.w();
    static_tf_pub_->sendTransform(tf);
    aligned_ = true;
    set_status("aligned: " + global_frame_ + " -> " + local_frame_ +
               " using " + alignment_mode_);
    publish_odom(*pending_base_, base_pub_);
    pending_base_.reset();
  }

  void publish_odom(const Odometry &input,
      const rclcpp::Publisher<Odometry>::SharedPtr &publisher)
  {
    if (!aligned_ || input.header.frame_id != local_frame_) return;
    Odometry output = input;
    output.header.frame_id = global_frame_;
    write_pose(global_from_local_ * pose_transform(input.pose.pose), output.pose.pose);
    rotate_pose_covariance(global_from_local_, output.pose.covariance);
    // nav_msgs/Odometry twist and its covariance remain in child_frame_id.
    publisher->publish(output);
  }

  void on_cloud(const Cloud &input)
  {
    if (!aligned_ || input.header.frame_id != local_frame_) return;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = input.header.stamp;
    transform.header.frame_id = global_frame_;
    transform.child_frame_id = local_frame_;
    const auto &translation = global_from_local_.getOrigin();
    const auto &rotation = global_from_local_.getRotation();
    transform.transform.translation.x = translation.x();
    transform.transform.translation.y = translation.y();
    transform.transform.translation.z = translation.z();
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();
    Cloud output;
    tf2::doTransform(input, output, transform);
    output.header.stamp = input.header.stamp;
    output.header.frame_id = global_frame_;
    cloud_pub_->publish(output);
  }

  std::string source_, alignment_mode_, global_frame_, local_frame_, status_;
  int64_t first_estimate_ns_ = 0, match_tolerance_ns_ = 0, settle_ns_ = 0;
  bool aligned_ = false;
  tf2::Transform global_from_local_{tf2::Transform::getIdentity()};
  tf2::Transform spawn_pose_{tf2::Transform::getIdentity()};
  std::optional<Odometry> pending_base_;
  std::deque<Odometry> truths_;
  rclcpp::Publisher<Odometry>::SharedPtr base_pub_, lidar_pub_;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr base_sub_, lidar_sub_, truth_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalNavigationAdapter>());
  rclcpp::shutdown();
  return 0;
}
