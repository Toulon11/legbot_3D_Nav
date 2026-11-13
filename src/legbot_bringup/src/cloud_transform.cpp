#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace {
using Cloud=sensor_msgs::msg::PointCloud2;

Cloud filter_near_points(const Cloud &input,double radius,std::size_t edge_columns,
                        std::size_t &near,std::size_t &invalid,std::size_t &edge) {
  std::array<std::size_t,3> offsets{};
  const std::array<const char*,3> names{"x","y","z"};
  for(std::size_t i=0;i<names.size();++i) {
    const auto field=std::find_if(input.fields.begin(),input.fields.end(),
        [&](const auto &f){return f.name==names[i];});
    if(field==input.fields.end() || field->datatype!=sensor_msgs::msg::PointField::FLOAT32 ||
       field->count!=1 || static_cast<std::uint64_t>(field->offset)+sizeof(float)>input.point_step)
      throw std::runtime_error("registered XYZ requires valid FLOAT32 coordinate fields");
    offsets[i]=field->offset;
  }
  const auto row_bytes=static_cast<std::uint64_t>(input.width)*input.point_step;
  const auto required=input.height?static_cast<std::uint64_t>(input.height-1)*input.row_step+row_bytes:0;
  if(input.row_step<row_bytes || input.data.size()<required)
    throw std::runtime_error("invalid PointCloud2 strides or data length");
  // tf2's PointCloud2 float iterators use native XYZ values as well.
  const std::uint16_t endian=1;
  if(input.is_bigendian==(*reinterpret_cast<const std::uint8_t*>(&endian)==1))
    throw std::runtime_error("non-native PointCloud2 byte order is unsupported");
  Cloud out=input;
  std::size_t kept=0;near=invalid=edge=0;
  const double limit=radius*radius;
  for(std::size_t row=0;row<input.height;++row) {
    for(std::size_t col=0;col<input.width;++col) {
      // Simulation scan-edge artifacts are masked in the organized sensor
      // image, before compaction/TF. Unorganized clouds have no scan columns.
      if(input.height>1 && (col<edge_columns || input.width-col<=edge_columns)) {++edge;continue;}
      const auto offset=row*input.row_step+col*input.point_step;
      std::array<float,3> xyz{};
      for(std::size_t i=0;i<xyz.size();++i)
        std::memcpy(&xyz[i],input.data.data()+offset+offsets[i],sizeof(float));
      if(!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) {++invalid;continue;}
      const double distance=static_cast<double>(xyz[0])*xyz[0]+static_cast<double>(xyz[1])*xyz[1]+
                            static_cast<double>(xyz[2])*xyz[2];
      if(distance<limit) {++near;continue;}
      std::memcpy(out.data.data()+kept*input.point_step,input.data.data()+offset,input.point_step);
      ++kept;
    }
  }
  out.height=1;out.width=kept;out.row_step=kept*input.point_step;
  out.data.resize(out.row_step);out.is_dense=true;
  return out;
}
}

class CloudTransform : public rclcpp::Node {
 tf2_ros::Buffer buffer_; tf2_ros::TransformListener listener_;
 rclcpp::Subscription<Cloud>::SharedPtr sub_;
 rclcpp::Publisher<Cloud>::SharedPtr pub_;
 rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
 std::string target_;
 double radius_;
 std::size_t edge_columns_;
 public:
 CloudTransform():Node("cloud_transform"),buffer_(get_clock()),listener_(buffer_) {
  target_=declare_parameter("target_frame",std::string("odom"));
  const auto input=declare_parameter("input_topic",std::string("/livox/points_raw"));
  const auto output=declare_parameter("output_topic",std::string("/livox/points_world"));
  const auto pose=declare_parameter("pose_topic",std::string("/go2/lidar_pose"));
  rcl_interfaces::msg::ParameterDescriptor geometry;geometry.read_only=true;
  geometry.description="Shared planning body radius; fallback for LiDAR minimum range.";
  const auto body_radius=declare_parameter<double>("robot_radius",geometry);
  if(!std::isfinite(body_radius) || body_radius<0)throw std::invalid_argument("robot_radius must be finite and nonnegative");
  rcl_interfaces::msg::ParameterDescriptor filtering;filtering.read_only=true;
  filtering.description="Minimum sensor-origin XYZ distance, independent of planning geometry.";
  radius_=declare_parameter<double>("lidar_min_range",body_radius,filtering);
  if(!std::isfinite(radius_) || radius_<0)throw std::invalid_argument("lidar_min_range must be finite and nonnegative");
  filtering.description="Columns removed from each horizontal edge of organized scans; zero disables.";
  const auto edges=declare_parameter<int>("lidar_edge_columns",0,filtering);
  if(edges<0)throw std::invalid_argument("lidar_edge_columns must be nonnegative");
  edge_columns_=static_cast<std::size_t>(edges);
  pose_pub_=create_publisher<nav_msgs::msg::Odometry>(pose,rclcpp::SensorDataQoS());
  pub_=create_publisher<Cloud>(output,rclcpp::SensorDataQoS());
  sub_=create_subscription<Cloud>(input,rclcpp::SensorDataQoS(),
   [this](Cloud::ConstSharedPtr input){
    if(input->header.frame_id.empty())return;
    try {
     auto t=buffer_.lookupTransform(target_,input->header.frame_id,input->header.stamp,rclcpp::Duration::from_seconds(0.05));
     std::size_t near=0,invalid=0,edge=0;
     auto filtered=filter_near_points(*input,radius_,edge_columns_,near,invalid,edge);
     nav_msgs::msg::Odometry pose;pose.header.stamp=input->header.stamp;pose.header.frame_id=target_;pose.child_frame_id=input->header.frame_id;
     pose.pose.pose.position.x=t.transform.translation.x;pose.pose.pose.position.y=t.transform.translation.y;pose.pose.pose.position.z=t.transform.translation.z;
     pose.pose.pose.orientation=t.transform.rotation;pose_pub_->publish(pose);
     Cloud out;
     if(filtered.width)tf2::doTransform(filtered,out,t);
     else {out=std::move(filtered);out.header.frame_id=target_;}
     out.header.stamp=input->header.stamp;pub_->publish(out);
     RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,
         "LiDAR filter: lidar_min_range=%.3f m, near=%zu invalid=%zu edge=%zu kept=%u",radius_,near,invalid,edge,out.width);
    }catch(const tf2::TransformException&e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Cloud TF unavailable: %s",e.what());}
     catch(const std::runtime_error&e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Invalid cloud: %s",e.what());}
   });
  RCLCPP_INFO(get_logger(),"LiDAR minimum XYZ range=%.3f m, planning robot_radius=%.3f m, edge_columns=%zu (%s -> %s)",
      radius_,body_radius,edge_columns_,input.c_str(),output.c_str());
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<CloudTransform>());rclcpp::shutdown();}
