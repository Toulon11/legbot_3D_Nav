#include "legbot_exploration/exploration_node_factories.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <deque>
#include <optional>
#include <mutex>
#include <shared_mutex>
#include <sstream>

namespace le=legbot_exploration;
namespace {
sensor_msgs::msg::PointCloud2 cloud_message(std::vector<std::uint8_t> data,
    const std_msgs::msg::Header &header,bool surfaces) {
  using Field=sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 out;out.header=header;out.height=1;
  out.is_bigendian=false;out.is_dense=false;out.point_step=surfaces?56:16;
  out.width=data.size()/out.point_step;out.row_step=out.width*out.point_step;out.data=std::move(data);
  const auto field=[&](const char *name,int offset,int type) {
    Field f;f.name=name;f.offset=offset;f.datatype=type;f.count=1;out.fields.push_back(f);
  };
  field("x",0,Field::FLOAT32);field("y",4,Field::FLOAT32);field("z",8,Field::FLOAT32);field("rgb",12,Field::FLOAT32);
  if(surfaces) {
    field("cost",16,Field::FLOAT32);field("ceiling_z",20,Field::FLOAT32);field("clearance",24,Field::FLOAT32);
    field("source",28,Field::UINT32);field("flags",32,Field::UINT32);field("slice_index",36,Field::UINT32);
    field("passage_cost",40,Field::FLOAT32);field("relief_cost",44,Field::FLOAT32);
    field("terrain_cost",48,Field::FLOAT32);field("inflated_terrain_cost",52,Field::FLOAT32);
  }
  return out;
}
double wall_now() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
}

class OnlineLayerMapNode : public rclcpp::Node {
  struct CloudInput {
    std::string topic,pose_topic;
    std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> poses;
    std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending;
    std::atomic<std::uint64_t> filtered_far{0},dropped_pose{0},matched{0};
    std::atomic<std::size_t> waiting_pose{0};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    std::atomic<std::uint64_t> seen{0},integrated{0};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  };
 public:
  explicit OnlineLayerMapNode(std::shared_ptr<le::SharedLayerMap> shared)
      :Node("online_layer_map"),shared_(shared?std::move(shared):std::make_shared<le::SharedLayerMap>()) {
    const auto root=declare_parameter<std::string>("pct_root",ament_index_cpp::get_package_share_directory("pct_planner"));
    const auto records=declare_parameter<int>("pct_max_surface_records",2000000);
    const auto cells=declare_parameter<int>("pct_max_map_cells",20000000);
    const double height=declare_parameter<double>("robot_reference_height",.32);
    std::ostringstream settings;
    settings<<'{';bool first=true;
    const auto number=[&](const char *key,const char *parameter,double value) {
      if(!first)settings<<',';first=false;
      const auto configured=declare_parameter<double>(parameter,value);
      settings<<'"'<<key<<"\":"<<configured;
      return configured;
    };
    const auto integer=[&](const char *key,const char *parameter,int value) {
      if(!first)settings<<',';first=false;
      settings<<'"'<<key<<"\":"<<declare_parameter<int>(parameter,value);
    };
    const auto text=[&](const char *key,const char *parameter,const char *value) {
      if(!first)settings<<',';first=false;
      settings<<'"'<<key<<"\":"<<std::quoted(declare_parameter<std::string>(parameter,value));
    };
    text("mapping_mode","pct_mapping_mode","static");
    integer("block_size","pct_block_size",32);
    integer("slice_update_workers","pct_slice_update_workers",0);
    integer("maturity_min_points","pct_maturity_min_points",3);
    shared_->surface_noise_tolerance=number("surface_noise_tolerance","pct_surface_noise_tolerance",.05);
    number("new_height_threshold","pct_new_height_threshold",.10);
    number("surface_normal_radius","pct_surface_normal_radius",.25);
    integer("surface_normal_min_points","pct_surface_normal_min_points",6);
    number("surface_normal_vertical","pct_surface_normal_vertical",.5);
    number("surface_normal_thickness","pct_surface_normal_thickness",.03);
    number("planning_budget_seconds","pct_planning_budget_seconds",1.5);
    number("observation_revisit_radius","pct_observation_revisit_radius",1.0);
    integer("floor_min_resolved_frontier_cells","pct_floor_min_resolved_frontier_cells",5);
    settings<<",\"fast_exploration\":"<<(declare_parameter<bool>("pct_fast_exploration",false)?"true":"false");
    integer("fast_min_visible_cells","pct_fast_min_visible_cells",3);
    integer("floor_failed_small_cluster_max_cells","pct_floor_failed_small_cluster_max_cells",0);
    settings<<",\"floor_global_review_enabled\":"
        <<(declare_parameter<bool>("pct_floor_global_review_enabled",true)?"true":"false");
    settings<<",\"floor_four_direction_search\":"
        <<(declare_parameter<bool>("pct_floor_four_direction_search",false)?"true":"false");
    number("floor_direction_search_timeout","pct_floor_direction_search_timeout_seconds",1.5);
    number("landing_support_radius","pct_landing_support_radius",.60);
    number("landing_min_area","pct_landing_min_area",1.50);
    number("landing_min_width","pct_landing_min_width",.80);
    number("landing_min_depth","pct_landing_min_depth",1.00);
    number("ground_lidar_x","ground_lidar_x",.35);
    number("ground_lidar_z","ground_lidar_z",.20);
    number("ground_lidar_pitch","ground_lidar_pitch",.6108652381980153);
    number("ground_lidar_horizontal_half_angle","ground_lidar_horizontal_half_angle",.7853981633974483);
    number("ground_lidar_elevation_min","ground_lidar_elevation_min",-.0872664625997165);
    number("ground_lidar_elevation_max","ground_lidar_elevation_max",.3490658503988659);
    number("ground_lidar_max_range","ground_lidar_max_range",12.);
    number("ground_lidar_min_range","lidar_min_range",.70);
    settings<<",\"observer_reference_height\":"<<height;
    number("relief_update_threshold","pct_relief_update_threshold",.25);
    number("terrain_update_threshold","pct_terrain_update_threshold",.25);
    number("inflation_update_threshold","pct_inflation_update_threshold",.25);
    integer("traversability_kernel_size","pct_traversability_kernel_size",7);
    integer("inflation_kernel_size","pct_inflation_kernel_size",13);
    number("standable_ratio","pct_standable_ratio",.5);
    shared_->xy_resolution=number("resolution","xy_resolution",.1);
    number("slice_dh","slice_height",.5);
    number("origin_x","pct_origin_x",0);number("origin_y","pct_origin_y",0);number("origin_z","pct_origin_z",0);
    number("robot_radius","robot_radius",.35);number("safety_margin","waypoint_safety_margin",.05);
    number("minimum_clearance","pct_min_clearance",.5);number("desired_clearance","pct_desired_clearance",.55);
    shared_->max_step_height=number("step_max","max_step_height",.16);
    settings<<",\"slope_max\":"<<std::tan(declare_parameter<double>("max_slope_radians",.4))<<'}';
    if(records<1 || cells<1 || !std::isfinite(height) || height<=0)throw std::invalid_argument("invalid PCT capacity/reference height");
    shared_->pct=std::make_shared<le::PctBackend>(root,height,records,cells,settings.str());
    frame_=declare_parameter<std::string>("frame_id","map");
    const auto cloud=declare_parameter<std::string>("cloud_topic","/slam/global_cloud_registered");
    const auto base=declare_parameter<std::string>("base_odom_topic","/slam/global_odometry_base");
    const auto period=declare_parameter<double>("publish_period",1.);
    show_=declare_parameter<bool>("publish_map_topics",true);
    max_display_=declare_parameter<int>("max_display_surfaces",120000);
    preview_=declare_parameter<bool>("graph_preview_enabled",true);
    graph_radius_=declare_parameter<double>("graph_search_radius",7.);
    graph_gain_=declare_parameter<double>("graph_min_height_gain",.07);
    graph_candidates_=declare_parameter<int>("graph_max_preview_candidates",8);
    graph_nodes_=declare_parameter<int>("graph_max_preview_nodes",4000);
    if(!std::isfinite(period)||period<=0||max_display_<1||!std::isfinite(graph_radius_)||graph_radius_<=0||
       !std::isfinite(graph_gain_)||graph_gain_<=0||graph_candidates_<1||graph_nodes_<1)
      throw std::invalid_argument("invalid PCT publication/preview settings");
    auto qos=rclcpp::QoS(1).transient_local().reliable();
    surface_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/exploration/layers/surfaces",qos);
    planning_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/exploration/layers/planning_surfaces",qos);
    returns_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/exploration/layers/measured_returns",qos);
    ceilings_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/exploration/layers/ceilings",qos);
    status_pub_=create_publisher<std_msgs::msg::String>("/exploration/layers/status",qos);
    graph_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/exploration/layers/graph_preview",qos);
    path_pub_=create_publisher<nav_msgs::msg::Path>("/exploration/layers/candidate_path",qos);
    cloud_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    reliable_range_=declare_parameter<double>("pct_reliable_observation_range",5.0);
    if(!std::isfinite(reliable_range_) || reliable_range_<0)throw std::invalid_argument("invalid PCT reliable observation range");
    add_input(cloud,declare_parameter<std::string>("cloud_sensor_odom_topic","/slam/odometry_lidar"));
    const auto auxiliary=declare_parameter<std::string>("auxiliary_cloud_topic","");
    if(!auxiliary.empty()) {
      if(auxiliary==cloud)throw std::invalid_argument("auxiliary LiDAR requires a distinct cloud topic");
      add_input(auxiliary,declare_parameter<std::string>("auxiliary_sensor_odom_topic","/ground_lidar/odometry_lidar"));
    }
    base_sub_=create_subscription<nav_msgs::msg::Odometry>(base,rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
      if(msg->header.frame_id!=frame_)return;
      base_={msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z};base_wall_=wall_now();
    });
    timer_=create_wall_timer(std::chrono::duration<double>(period),[this]{publish_map();});
    RCLCPP_INFO(get_logger(),"Online PCT map: %zu LiDAR inputs, registered XYZ only",inputs_.size());
  }
 private:
  static std::int64_t stamp(const builtin_interfaces::msg::Time &t) {
    return static_cast<std::int64_t>(t.sec)*1000000000LL+t.nanosec;
  }
  void drain_input(CloudInput &input) {
    for(auto it=input.pending.begin();it!=input.pending.end();) {
      const auto ns=stamp((*it)->header.stamp);
      const auto found=std::find_if(input.poses.begin(),input.poses.end(),
          [&](const auto &pose){return stamp(pose->header.stamp)==ns;});
      if(found!=input.poses.end()) {
        const auto cloud=*it;it=input.pending.erase(it);
        const auto &p=(*found)->pose.pose.position;
        ++input.matched;on_cloud(*cloud,input,le::Point{p.x,p.y,p.z});
      } else if(!input.poses.empty() && ns<stamp(input.poses.back()->header.stamp)-2000000000LL) {
        ++input.dropped_pose;it=input.pending.erase(it);
      } else ++it;
    }
    input.waiting_pose.store(input.pending.size());
  }
  void add_input(const std::string &cloud,const std::string &pose_topic) {
    auto owned=std::make_unique<CloudInput>();auto *input=owned.get();input->topic=cloud;input->pose_topic=pose_topic;
    rclcpp::SubscriptionOptions options;options.callback_group=cloud_group_;
    if(reliable_range_>0)input->pose_sub=create_subscription<nav_msgs::msg::Odometry>(pose_topic,rclcpp::SensorDataQoS(),
      [this,input](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto &p=msg->pose.pose.position;
        if(msg->header.frame_id!=frame_ || stamp(msg->header.stamp)<=0 ||
           !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return;
        input->poses.push_back(msg);if(input->poses.size()>128)input->poses.pop_front();
        drain_input(*input);
      },options);
    input->cloud_sub=create_subscription<sensor_msgs::msg::PointCloud2>(cloud,rclcpp::SensorDataQoS(),
      [this,input](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
        ++seen_;++input->seen;
        if(msg->header.frame_id!=frame_){++wrong_frame_;return;}
        if(msg->width && msg->height)shared_->last_sensor_wall_seconds.store(wall_now());
        if(shared_->planning_active.load()){++shared_->dropped_planning;return;}
        if(reliable_range_<=0)on_cloud(*msg,*input,std::nullopt);
        else {
          if(stamp(msg->header.stamp)<=0){++input->dropped_pose;return;}
          input->pending.push_back(msg);
          if(input->pending.size()>5){input->pending.pop_front();++input->dropped_pose;}
          drain_input(*input);
        }
      },options);
    inputs_.push_back(std::move(owned));
  }
  void on_cloud(const sensor_msgs::msg::PointCloud2 &msg,CloudInput &input,const std::optional<le::Point> &origin) {
    std::vector<le::LayerPoint> points;points.reserve(static_cast<std::size_t>(msg.width)*msg.height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(msg,"x"),y(msg,"y"),z(msg,"z");
      for(;x!=x.end();++x,++y,++z)if(std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*z)) {
        if(origin && std::pow(*x-(*origin)[0],2)+std::pow(*y-(*origin)[1],2)+std::pow(*z-(*origin)[2],2)>reliable_range_*reliable_range_) {++input.filtered_far;continue;}
        points.push_back({*x,*y,*z});
      }
    }catch(const std::runtime_error &e){++invalid_cloud_;RCLCPP_WARN(get_logger(),"Invalid registered XYZ: %s",e.what());return;}
    if(points.empty())return;
    shared_->last_sensor_wall_seconds.store(wall_now());
    if(shared_->planning_active.load()){++shared_->dropped_planning;return;}
    const auto start=wall_now();
    std::unique_lock<std::shared_mutex> lock(shared_->mutex,std::try_to_lock);
    if(!lock.owns_lock()||shared_->planning_active.load()){++shared_->dropped_planning;return;}
    shared_->pct->append(points,origin);
    if(!shared_->pct->last_error().empty()) {
      ++rejected_;RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"PCT integration failed: %s",shared_->pct->last_error().c_str());return;
    }
    ++input.integrated;
    ++received_;integrated_+=points.size();last_raw_=points.size();++shared_->current_version;
    shared_->last_integration_wall_seconds.store(wall_now());
    integration_ms_=1000*(wall_now()-start);
  }
  void publish_map() {
    const auto started=wall_now();
    if(shared_->planning_active.load())return;
    le::PctInspection inspection;std::string metadata;std::uint64_t version;bool ready=false;
    double lock_wait_ms=0,commit_ms=0,inspection_ms=0,locked_ms=0;
    std::size_t surface_records=0;
    {
      std::unique_lock<std::shared_mutex> lock(shared_->mutex);
      const auto acquired=wall_now();lock_wait_ms=1000*(acquired-started);
      if(shared_->planning_active.load())return;
      version=shared_->current_version.load();
      if(shared_->pct->point_count())ready=shared_->pct->refresh(version);
      const auto committed=wall_now();commit_ms=1000*(committed-acquired);
      if(ready) {
        shared_->published_wall_seconds.store(wall_now());
        shared_->committed_version.store(version);
        inspection=shared_->pct->inspect(version,base_,show_,max_display_,
            preview_&&base_wall_>0&&wall_now()-base_wall_<2.,graph_radius_,graph_gain_,graph_candidates_,graph_nodes_);
      }
      metadata=shared_->pct->map_metadata();
      surface_records=shared_->pct->point_count();
      inspection_ms=1000*(wall_now()-committed);locked_ms=1000*(wall_now()-acquired);
    }
    std_msgs::msg::Header header;header.frame_id=frame_;header.stamp=now();
    if(show_ && inspection.valid) {
      surface_pub_->publish(cloud_message(std::move(inspection.display),header,true));
      planning_pub_->publish(cloud_message(std::move(inspection.surfaces),header,true));
      returns_pub_->publish(cloud_message(std::move(inspection.returns),header,false));
      ceilings_pub_->publish(cloud_message(std::move(inspection.ceilings),header,false));
    }
    if(show_)publish_preview(inspection,header);
    std::ostringstream json;
    json<<"{\"backend\":\"project_pct_fixed_online\",\"version\":"<<version
      <<",\"reliable_observation_range\":"<<reliable_range_
      <<",\"pct_version\":"<<version<<",\"pct_ready\":"<<(ready?"true":"false")
      <<",\"pct_surface_records\":"<<surface_records
      <<",\"cloud_messages_seen\":"<<seen_<<",\"clouds_received\":"<<received_
      <<",\"integrated_points\":"<<integrated_<<",\"rejected_frames\":"<<rejected_
      <<",\"wrong_frame\":"<<wrong_frame_<<",\"invalid_cloud\":"<<invalid_cloud_
      <<",\"dropped_planning\":"<<shared_->dropped_planning.load()
      <<",\"last_sensor_age\":"<<(shared_->last_sensor_wall_seconds.load()>0?wall_now()-shared_->last_sensor_wall_seconds.load():-1.)
      <<",\"last_integration_age\":"<<(shared_->last_integration_wall_seconds.load()>0?wall_now()-shared_->last_integration_wall_seconds.load():-1.)
      <<",\"committed_input_version\":"<<shared_->committed_version.load()
      <<",\"last_raw_cloud_points\":"<<last_raw_<<",\"integration_ms\":"<<integration_ms_
      <<",\"pct_map\":"<<metadata<<",\"graph_state\":\""<<inspection.preview_state
      <<"\",\"graph_path_points\":"<<inspection.path.size()
      <<",\"inspection_valid\":"<<(inspection.valid?"true":"false")
      <<",\"lock_wait_ms\":"<<lock_wait_ms<<",\"commit_ms\":"<<commit_ms
      <<",\"inspection_ms\":"<<inspection_ms<<",\"lock_held_ms\":"<<locked_ms
      <<",\"read_wait_ms\":"<<shared_->last_read_wait_ms.load()
      <<",\"read_commit_ms\":"<<shared_->last_read_commit_ms.load()
      <<",\"read_phase_ms\":"<<shared_->last_read_phase_ms.load()
      <<",\"planning_ms\":"<<shared_->last_planning_ms.load()
      <<",\"update_ms\":"<<1000*(wall_now()-started)<<",\"cloud_sources\":[";
    for(std::size_t i=0;i<inputs_.size();++i) {
      if(i)json<<',';const auto &input=*inputs_[i];
      json<<"{\"topic\":"<<std::quoted(input.topic)<<",\"seen\":"<<input.seen.load()
          <<",\"integrated\":"<<input.integrated.load()
          <<",\"sensor_pose_topic\":"<<std::quoted(input.pose_topic)
          <<",\"matched_pose_frames\":"<<input.matched.load()<<",\"waiting_pose_frames\":"<<input.waiting_pose.load()
          <<",\"dropped_pose_frames\":"<<input.dropped_pose.load()<<",\"filtered_far_points\":"<<input.filtered_far.load()<<'}';
    }
    json<<"]}";
    status_pub_->publish(std_msgs::msg::String().set__data(json.str()));
  }
  void publish_preview(const le::PctInspection &data,const std_msgs::msg::Header &header) {
    using Marker=visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;Marker clear;clear.header=header;clear.action=Marker::DELETEALL;array.markers.push_back(clear);
    const auto marker=[&](int id,int type,double r,double g,double b,double size,const std::vector<le::Point> &points) {
      Marker m;m.header=header;m.ns="pct_ascent_preview";m.id=id;m.type=type;m.action=Marker::ADD;
      m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=size;m.color.r=r;m.color.g=g;m.color.b=b;m.color.a=.8;
      for(const auto &p:points){geometry_msgs::msg::Point q;q.x=p[0];q.y=p[1];q.z=p[2]+.03;m.points.push_back(q);}array.markers.push_back(std::move(m));
    };
    marker(0,Marker::POINTS,.5,.7,.8,.035,data.preview_nodes);
    marker(1,Marker::SPHERE_LIST,1.,.65,0.,.11,data.preview_candidates);
    marker(2,Marker::LINE_STRIP,1.,.1,.9,.06,data.path);graph_pub_->publish(array);
    nav_msgs::msg::Path path;path.header=header;
    for(const auto &p:data.path){geometry_msgs::msg::PoseStamped pose;pose.header=header;pose.pose.orientation.w=1.;
      pose.pose.position.x=p[0];pose.pose.position.y=p[1];pose.pose.position.z=p[2];path.poses.push_back(pose);}path_pub_->publish(path);
  }
  std::shared_ptr<le::SharedLayerMap> shared_;
  std::vector<std::unique_ptr<CloudInput>> inputs_;
  le::Point base_{};double base_wall_=0,graph_radius_=7,graph_gain_=.07,reliable_range_=5.;
  std::atomic<double> integration_ms_{0};
  int max_display_=120000,graph_candidates_=8,graph_nodes_=4000;
  bool show_=true,preview_=true;std::string frame_;
  std::atomic<std::uint64_t> seen_{0},received_{0},rejected_{0},integrated_{0},wrong_frame_{0},invalid_cloud_{0};
  std::atomic<std::size_t> last_raw_{0};
  rclcpp::CallbackGroup::SharedPtr cloud_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr base_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_pub_,planning_pub_,returns_pub_,ceilings_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
std::shared_ptr<rclcpp::Node> make_online_layer_map_node(std::shared_ptr<le::SharedLayerMap> shared) {
  return std::make_shared<OnlineLayerMapNode>(std::move(shared));
}
