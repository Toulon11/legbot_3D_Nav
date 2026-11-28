#include "legbot_exploration/exploration_node_factories.hpp"
#include "legbot_exploration/shared_layer_map.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <unistd.h>

namespace le=legbot_exploration;
using namespace std::chrono_literals;
int main() {
  const auto domain=std::to_string(150+getpid()%70);
  setenv("ROS_DOMAIN_ID",domain.c_str(),1);
  const char *args[]={"test_map_publication","--ros-args","-p","publish_period:=0.05",
    "-p","full_snapshot_period:=0.15","-p","graph_preview_enabled:=false",
    "-p","pct_maturity_min_points:=1","-p","pct_reliable_observation_range:=0.0"};
  rclcpp::init(sizeof(args)/sizeof(*args),args);
  auto shared=std::make_shared<le::SharedLayerMap>();
  auto map=make_online_layer_map_node(shared);
  std::vector<le::LayerPoint> points;
  for(int x=-10;x<=10;++x)for(int y=-10;y<=10;++y)
    for(double z:{0.,1.5})points.push_back({x*.1,y*.1,z});
  shared->pct->append(points);
  assert(shared->pct->last_error().empty());
  assert(shared->pct->refresh(1));shared->current_version=1;shared->committed_version=1;
  // A planner reservation and its reader-yield window must not prevent a
  // coherent export when the actual map lock is free. The former publisher
  // returned before taking any snapshot forever in this state.
  shared->planning_active=1;
  shared->reader_yield_until=std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count()+3600.;
  auto observer=std::make_shared<rclcpp::Node>("map_publication_fixture");
  auto qos=rclcpp::QoS(10).transient_local().reliable();
  std::atomic<unsigned> surfaces{0},returns{0},ceilings{0},updates{0},statuses{0};
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs;
  const auto subscribe=[&](const char *topic,std::atomic<unsigned> &count,unsigned step) {
    subs.push_back(observer->create_subscription<sensor_msgs::msg::PointCloud2>(topic,qos,
      [&,step,counter=&count](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
        assert(msg->width>0 && msg->height==1 && msg->point_step==step);
        assert(msg->data.size()==msg->width*step);assert(msg->header.frame_id=="map");
        sensor_msgs::PointCloud2ConstIterator<float> z(*msg,"z");
        for(;z!=z.end();++z)assert(std::isfinite(*z));
        ++*counter;
      }));
  };
  subscribe("/exploration/layers/surfaces",surfaces,64);
  subscribe("/exploration/layers/measured_returns",returns,16);
  subscribe("/exploration/layers/ceilings",ceilings,16);
  auto delta=observer->create_subscription<std_msgs::msg::String>("/exploration/layers/surface_updates",qos,
    [&](std_msgs::msg::String::ConstSharedPtr msg){assert(msg->data.find("\"version\":1")!=std::string::npos);++updates;});
  auto status=observer->create_subscription<std_msgs::msg::String>("/exploration/layers/status",qos,
    [&](std_msgs::msg::String::ConstSharedPtr msg){
      assert(msg->data.find("\"inspection_valid\":true")!=std::string::npos);++statuses;
    });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{},2);
  executor.add_node(map);executor.add_node(observer);
  const auto end=std::chrono::steady_clock::now()+4s;
  while(std::chrono::steady_clock::now()<end && (surfaces<3 || returns<3 || ceilings<3 || updates<5)) {
    executor.spin_some();std::this_thread::sleep_for(5ms);
  }
  assert(surfaces>=3 && returns>=3 && ceilings>=3 && updates>=5 && statuses>=5);
  assert(shared->planning_active==1); // Export must not impersonate a planner.
  std::cout<<"PASS repeated nonempty surfaces/returns/ceilings and coverage export during planner reservation and reader-yield window\n";
  executor.remove_node(observer);executor.remove_node(map);
  shared->planning_active=0;map.reset();observer.reset();rclcpp::shutdown();
}
