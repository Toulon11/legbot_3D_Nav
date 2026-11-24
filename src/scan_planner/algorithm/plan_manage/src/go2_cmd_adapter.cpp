#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <sstream>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <legbot_runtime/runtime.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <legbot_runtime/transforms.hpp>

#include "bspline_opt/uniform_bspline.h"
#include <scan_planner/msg/bspline.hpp>
#include <plan_manage/goal_tolerance.h>

namespace
{
using scan_planner::UniformBspline;

legbot::Publisher cmd_vel_pub, stop_status_pub;
legbot::Publisher execution_frozen_pub;
legbot::Publisher goal_stop_ack_pub;
legbot::Publisher goal_failure_pub, tolerance_status_pub;
legbot::Publisher reverse_result_pub, turn_result_pub, recovery_state_pub;
legbot::Subscriber bspline_sub;
legbot::Subscriber odom_sub;
legbot::Subscriber goal_sub, goal_request_sub;
legbot::Subscriber turn_request_sub, action_cancel_sub, action_enabled_sub;
legbot::Subscriber reverse_request_sub;
legbot::Timer cmd_timer;

bool receive_traj = false;
bool have_odom = false;
legbot::Time last_odom_time;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
double last_nearest_time = 0.0;
int traj_id = 0;
legbot::Time latest_goal_time;
legbot::Time latest_traj_start;
bool have_goal_time = false;
bool have_goal_position = false;
bool goal_stop_latched = false;
bool goal_ack_sent = false;
bool execution_failure_sent = false;
enum class NavigationPhase { waiting, translating, rotating };
NavigationPhase navigation_phase = NavigationPhase::waiting;
bool navigation_started = false;
std::chrono::steady_clock::time_point goal_accepted_at;
double first_trajectory_timeout = 5.;
bool trajectory_has_motion = false;
legbot::Time navigation_started_at, navigation_progress_at;
double best_navigation_distance = std::numeric_limits<double>::infinity();
double best_navigation_yaw_error = std::numeric_limits<double>::infinity();
Eigen::Vector2d navigation_progress_position = Eigen::Vector2d::Zero();
double navigation_progress_yaw = 0.0;
geometry_msgs::msg::PoseStamped latest_goal_msg;
Eigen::Vector3d latest_goal_pos = Eigen::Vector3d::Zero();

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;

double time_forward;
double heading_error_threshold;
double rotate_exit_ratio;
double kp_pos;
double kp_yaw;
double max_vx;
double max_vy;
double max_vyaw;
double finish_dist;
double default_finish_dist;
scan_planner::msg::GoalRequest current_goal_request;
double finish_dist_z;
bool use_goal_z;
double tracking_z_weight;
bool steer_with_position_error;
double odom_timeout;
double slope_up_threshold;
double slope_down_threshold;
double stair_speed_scale;
double min_stair_speed;
double min_walk_speed;
double goal_slowdown_distance;
double terminal_failure_seconds;
double terminal_rotate_enter;
double terminal_rotate_exit;
double execution_timeout;
double turn_slowdown_angle;
double min_turn_speed_scale;
double cmd_accel_limit;
double cmd_yaw_accel_limit;
std::string body_pose_topic;
geometry_msgs::msg::Twist last_motion_cmd;
legbot::Time last_motion_cmd_time;
enum class StopOwner { none, goal, turn, reverse };
StopOwner stop_owner=StopOwner::none;
legbot::Time stop_started, stop_stable_since, stop_status_at;
double stop_deceleration=.35,stop_yaw_deceleration=1.,stop_stable_seconds=.30;
double stop_linear_threshold=.03,stop_angular_threshold=.10,stop_timeout=5.;
constexpr double kTerminalProgressDistance=.01;
struct MotionSample {double t;Eigen::Vector3d p;Eigen::Quaterniond q;};
std::deque<MotionSample> motion_samples;
double measured_linear_speed=std::numeric_limits<double>::infinity();
double measured_angular_speed=std::numeric_limits<double>::infinity();
Eigen::Vector3d measured_velocity=Eigen::Vector3d::Zero(),measured_rotation_rate=Eigen::Vector3d::Zero();
bool rotate_only = false;
int rotate_direction = 0;
// Recovery actions own their execution watchdogs. The gait coordinator is the
// only mode publisher; a matching, fresh enable lease gates all action motion.
bool turn_active = false, action_enabled = false;
std::uint64_t turn_stamp = 0, last_action_stamp = 0;
Eigen::Vector2d turn_start = Eigen::Vector2d::Zero();
double turn_angle = 0.0, turn_progress = 0.0;
double turn_speed = 0.35, turn_timeout = 8.0;
legbot::Time turn_started_at, action_requested_at, action_enable_at;
bool action_started = false;

bool reverse_active = false;
std::uint64_t reverse_stamp = 0;
Eigen::Vector2d reverse_start = Eigen::Vector2d::Zero();
Eigen::Vector2d reverse_direction = Eigen::Vector2d::Zero();
bool recovery_shift = false;
double reverse_yaw = 0.0;
double reverse_distance = 0.0;
double reverse_best_progress = 0.0;
legbot::Time reverse_started_at;
legbot::Time reverse_progress_at;
double reverse_speed = 0.12;
double shift_speed = 0.22;
constexpr double shift_stop_lead = 0.07;

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
{
  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-6)
    return value;
  return value / norm * max_norm;
}

void loadParams(const legbot::NodeHandle &nh)
{
  nh.param("time_forward", time_forward, 0.55);
  nh.param("heading_error_threshold", heading_error_threshold, 1.20);
  nh.param("rotate_exit_ratio", rotate_exit_ratio, 0.75);
  nh.param("kp_pos", kp_pos, 0.90);
  nh.param("kp_yaw", kp_yaw, 1.20);
  nh.param("max_vx", max_vx, 0.50);
  nh.param("max_vy", max_vy, 0.25);
  nh.param("max_vyaw", max_vyaw, 1.00);
  nh.param("finish_dist", default_finish_dist, 0.25);
  finish_dist=default_finish_dist;
  nh.param("finish_dist_z", finish_dist_z, 0.20);
  nh.param("use_goal_z", use_goal_z, false);
  // A local B-spline spans only one short, monotonic route segment.  Its
  // nearest-point tracker must not inherit the large Z weight used by the
  // global multi-floor projection: PCT reference height is intentionally
  // above the measured base pose and would pin tracking at t=0 on stairs.
  nh.param("tracking_z_weight", tracking_z_weight, 0.0);
  nh.param("steer_with_position_error", steer_with_position_error, false);
  nh.param("odom_timeout", odom_timeout, 0.5);
  nh.param("slope_up_threshold", slope_up_threshold, 0.30);
  nh.param("slope_down_threshold", slope_down_threshold, -0.30);
  nh.param("stair_speed_scale", stair_speed_scale, 1.00);
  nh.param("min_stair_speed", min_stair_speed, 0.30);
  nh.param("min_walk_speed", min_walk_speed, 0.0);
  nh.param("goal_slowdown_distance", goal_slowdown_distance, 0.80);
  nh.param("terminal_failure_seconds", terminal_failure_seconds, 8.0);
  nh.param("terminal_rotate_enter", terminal_rotate_enter, 0.50);
  nh.param("terminal_rotate_exit", terminal_rotate_exit, 0.30);
  nh.param("execution_timeout", execution_timeout, 600.0);
  nh.param("first_trajectory_timeout", first_trajectory_timeout, 5.0);
  if(!std::isfinite(first_trajectory_timeout) || first_trajectory_timeout<=0.)
    first_trajectory_timeout=5.;
  nh.param("recovery_reverse_speed", reverse_speed, 0.12);
  nh.param("recovery_shift_speed", shift_speed, 0.22);
  nh.param("recovery_turn_speed", turn_speed, 0.35);
  nh.param("recovery_turn_timeout", turn_timeout, 8.0);
  nh.param("turn_slowdown_angle", turn_slowdown_angle, 0.30);
  nh.param("min_turn_speed_scale", min_turn_speed_scale, 0.30);
  nh.param("cmd_accel_limit", cmd_accel_limit, 0.75);
  nh.param("cmd_yaw_accel_limit", cmd_yaw_accel_limit, 2.00);
  nh.param("stop_deceleration",stop_deceleration,.35);
  nh.param("stop_yaw_deceleration",stop_yaw_deceleration,1.);
  nh.param("stop_stable_seconds",stop_stable_seconds,.30);
  nh.param("stop_max_linear_speed",stop_linear_threshold,.03);
  nh.param("stop_max_angular_speed",stop_angular_threshold,.10);
  nh.param("stop_timeout",stop_timeout,5.);
  if(!std::isfinite(min_walk_speed)||min_walk_speed<0)throw std::invalid_argument("invalid minimum walk speed");
  for(double value:{stop_deceleration,stop_yaw_deceleration,stop_stable_seconds,stop_linear_threshold,
                    stop_angular_threshold,stop_timeout})
    if(!std::isfinite(value)||value<=0)throw std::invalid_argument("invalid controlled stop configuration");
  legbot::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/Odometry_gazebo"));

  max_vyaw = std::min(std::max(max_vyaw, 0.0), 1.0);
  max_vx = std::min(std::max(max_vx, 0.0), 0.75);
  terminal_failure_seconds = clamp(terminal_failure_seconds, 2.0, 60.0);
  terminal_rotate_enter = clamp(terminal_rotate_enter, 0.05, M_PI);
  terminal_rotate_exit = clamp(terminal_rotate_exit, 0.01, terminal_rotate_enter - 0.01);
  execution_timeout = clamp(execution_timeout, 2.0, 3600.0);
  reverse_speed = clamp(reverse_speed, 0.05, 0.18);
  shift_speed = clamp(shift_speed, 0.10, 0.25);
  turn_speed = clamp(turn_speed, 0.05, 0.60);
  turn_timeout = clamp(turn_timeout, 2.0, 60.0);
  heading_error_threshold = clamp(heading_error_threshold, 0.10, M_PI);
  min_turn_speed_scale = clamp(min_turn_speed_scale, 0.0, 1.0);
  rotate_exit_ratio = clamp(rotate_exit_ratio, 0.0, 1.0);

  // This adapter commands a holonomic GO2 interface (linear.x + linear.y).
  // Pointing body yaw at the cross-track correction turns a lateral path
  // correction into a large arc and becomes singular at the final point.
  // Keep accepting the legacy launch argument for command-line compatibility,
  // but do not let an old copied command reactivate that unstable behaviour.
  if (steer_with_position_error)
  {
    ROS_WARN("[scan_go2_cmd_adapter] steer_with_position_error=true is incompatible with "
             "holonomic GO2 tracking; forcing spline-tangent steering.");
    steer_with_position_error = false;
  }
}

double findNearestTime(UniformBspline &pos_traj, const Eigen::Vector3d &pos,
                       double t_min, double t_max)
{
  t_min = std::max(t_min, 0.0);
  t_max = std::min(t_max, pos_traj.getTimeSum());

  double best_t = t_min;
  double best_dist = std::numeric_limits<double>::max();
  const double step = 0.01;

  for (double t = t_min; t <= t_max + 1e-6; t += step)
  {
    const double tc = std::min(t, t_max);
    const Eigen::Vector3d p = pos_traj.evaluateDeBoorT(tc);
    const double dist_xy = (p.head<2>() - pos.head<2>()).squaredNorm();
    const double dist_z = p.z() - pos.z();
    const double dist = dist_xy + tracking_z_weight * dist_z * dist_z;
    if (dist < best_dist)
    {
      best_dist = dist;
      best_t = tc;
    }
  }

  return best_t;
}

double estimateDesiredYaw(double t_near, double t_look, const Eigen::Vector3d &pos_near,
                          const Eigen::Vector3d &pos_look)
{
  const Eigen::Vector3d vel = traj[1].evaluateDeBoorT(t_look);
  if (vel.head<2>().norm() > 0.08)
    return std::atan2(vel(1), vel(0));

  const Eigen::Vector3d dir = pos_look - pos_near;
  if (dir.head<2>().norm() > 1e-4)
    return std::atan2(dir(1), dir(0));

  return odom_yaw;
}

void publishStop(double vyaw = 0.0)
{
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  cmd_vel_pub.publish(cmd);
  last_motion_cmd = cmd;
  last_motion_cmd_time = legbot::Time::now();
}

double approach(double current, double target, double maximum_delta)
{
  return current + clamp(target - current, -maximum_delta, maximum_delta);
}

void publishSmooth(const geometry_msgs::msg::Twist &target,double linear_limit=-1.,double yaw_limit=-1.)
{
  const legbot::Time now = legbot::Time::now();
  double dt = last_motion_cmd_time.isZero() ? 0.01 : (now - last_motion_cmd_time).toSec();
  dt = clamp(dt, 0.001, 0.05);
  geometry_msgs::msg::Twist cmd = target;
  // Use the same braking limits before and inside the arrival circle.
  const Eigen::Vector2d previous(last_motion_cmd.linear.x,last_motion_cmd.linear.y);
  const Eigen::Vector2d desired(target.linear.x,target.linear.y);
  const bool braking=desired.norm()<previous.norm() || desired.dot(previous)<0.;
  const bool yaw_braking=std::abs(target.angular.z)<std::abs(last_motion_cmd.angular.z) || target.angular.z*last_motion_cmd.angular.z<0.;
  const double linear_delta=std::max(0.,linear_limit<0?(braking?std::min(cmd_accel_limit,stop_deceleration):cmd_accel_limit):linear_limit)*dt;
  const double yaw_delta=std::max(0.,yaw_limit<0?(yaw_braking?std::min(cmd_yaw_accel_limit,stop_yaw_deceleration):cmd_yaw_accel_limit):yaw_limit)*dt;
  const Eigen::Vector2d delta=clampNorm(Eigen::Vector2d(desired-previous),linear_delta);
  cmd.linear.x=last_motion_cmd.linear.x+delta.x();cmd.linear.y=last_motion_cmd.linear.y+delta.y();
  cmd.angular.z = approach(last_motion_cmd.angular.z, target.angular.z, yaw_delta);
  cmd_vel_pub.publish(cmd);
  last_motion_cmd = cmd;
  last_motion_cmd_time = now;
}

void publishExecutionFrozen(bool frozen)
{
  std_msgs::msg::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

void failNavigation(const std::string &reason)
{
  if (execution_failure_sent) return;
  execution_failure_sent = true;
  stop_owner=StopOwner::none;
  publishExecutionFrozen(false);
  publishStop();
  const auto &stamp = latest_goal_msg.header.stamp;
  const std::uint64_t stamp_ns = static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL +
                                 stamp.nanosec;
  if (stamp_ns != 0) {
    std_msgs::msg::String failure;
    failure.data = std::to_string(stamp_ns) + "|" + reason;
    goal_failure_pub.publish(failure);
  }
  ROS_WARN("[scan_go2_cmd_adapter] Navigation stopped: %s.", reason.c_str());
}

bool navigationMakingProgress(bool rotating, bool terminal, double distance,
                              double yaw_error)
{
  if (!have_goal_position) return true;
  const auto now = legbot::Time::now();
  const auto phase = rotating ? NavigationPhase::rotating : NavigationPhase::translating;
  if (navigation_phase != phase) {
    navigation_phase = phase;
    navigation_progress_at = now;
    best_navigation_distance = distance;
    best_navigation_yaw_error = yaw_error;
    navigation_progress_position = odom_pos.head<2>();
    navigation_progress_yaw = odom_yaw;
  }
  bool progress = false;
  if (rotating) {
    // A replacement spline alone must not count as physical yaw progress.
    progress = yaw_error <= best_navigation_yaw_error - 0.02 &&
        std::abs(normalizeAngle(odom_yaw - navigation_progress_yaw)) >= 0.01;
  } else if (terminal) {
    progress = distance <= best_navigation_distance - kTerminalProgressDistance;
  } else {
    // A valid detour can temporarily increase distance to the final goal.
    progress = (odom_pos.head<2>() - navigation_progress_position).norm() >= 0.02;
  }
  if (progress) {
    navigation_progress_at = now;
    best_navigation_distance = distance;
    best_navigation_yaw_error = yaw_error;
    navigation_progress_position = odom_pos.head<2>();
    navigation_progress_yaw = odom_yaw;
  }
  if ((now - navigation_progress_at).toSec() >= terminal_failure_seconds) {
    failNavigation(rotating ? "execution_heading_no_progress" : "execution_no_progress");
    return false;
  }
  return true;
}

void publishActionState(std::uint64_t stamp, const std::string &state)
{
  std_msgs::msg::String msg;
  msg.data = std::to_string(stamp) + "|" + state;
  recovery_state_pub.publish(msg);
}

void finishTurn(const std::string &reason, bool success)
{
  if (!turn_active) return;
  turn_active = false;
  stop_owner=StopOwner::none;
  action_enabled = false;
  publishExecutionFrozen(false);
  publishStop();
  publishActionState(turn_stamp, "finished");
  std_msgs::msg::String result;
  result.data = std::to_string(turn_stamp) + "|" + reason;
  if (success) turn_result_pub.publish(result);
  else goal_failure_pub.publish(result);
}

void finishReverse(const std::string &reason, bool success)
{
  if (!reverse_active) return;
  reverse_active = false;
  stop_owner=StopOwner::none;
  action_enabled = false;
  publishActionState(reverse_stamp, "finished");
  publishExecutionFrozen(false);
  publishStop();
  std_msgs::msg::String result;
  result.data = std::to_string(reverse_stamp) + "|" + reason;
  if (success)
    reverse_result_pub.publish(result);
  else
    goal_failure_pub.publish(result);
}

void reverseRequestCallback(const std_msgs::msg::String::ConstSharedPtr &msg)
{
  if (!msg) return;
  std::istringstream input(msg->data);
  std::uint64_t stamp = 0;
  double sx = 0.0, sy = 0.0, tx = 0.0, ty = 0.0, yaw = 0.0;
  char a = 0, b = 0, c = 0, d = 0, e = 0;
  if (!(input >> stamp >> a >> sx >> b >> sy >> c >> tx >> d >> ty >> e >> yaw) ||
      a != '|' || b != '|' || c != '|' || d != '|' || e != '|' ||
      stamp == 0 || !std::isfinite(sx) || !std::isfinite(sy) ||
      !std::isfinite(tx) || !std::isfinite(ty) || !std::isfinite(yaw)) return;
  input >> std::ws;
  bool shift=false;
  if (!input.eof()) {
    char separator;
    std::string kind;
    if(!(input >> separator >> kind) || separator!='|' || kind!="shift") return;
    input >> std::ws;
    if(!input.eof()) return;
    shift=true;
  }
  if (stamp <= last_action_stamp ||
      (have_goal_time && stamp < static_cast<std::uint64_t>(latest_goal_time.value.nanoseconds()))) return;
  last_action_stamp = stamp;
  if (reverse_active) finishReverse("reverse_preempted", false);
  if (turn_active) finishTurn("turn_preempted", false);
  stop_owner=StopOwner::none;
  reverse_stamp = stamp;
  const Eigen::Vector2d start(sx, sy), target(tx, ty);
  const Eigen::Vector2d rear(-std::cos(yaw), -std::sin(yaw));
  const Eigen::Vector2d displacement = target - start;
  const double planned_distance = shift?displacement.norm():displacement.dot(rear);
  const legbot::Time now = legbot::Time::now();
  if (!have_odom || (now - last_odom_time).toSec() > odom_timeout ||
      (odom_pos.head<2>() - start).norm() > (shift?.04:.15) ||
      std::abs(normalizeAngle(odom_yaw - yaw)) > 0.25 ||
      planned_distance < 0.20-1e-6 || planned_distance > 0.60+1e-6 ||
      (!shift && (displacement - planned_distance * rear).norm() > 0.05)) {
    receive_traj = false;
    execution_failure_sent = true;
    publishExecutionFrozen(false);
    publishStop();
    std_msgs::msg::String failure;
    failure.data = std::to_string(stamp) + (shift?"|shift_request_invalid":"|reverse_request_invalid");
    goal_failure_pub.publish(failure);
    return;
  }
  reverse_start = start;
  reverse_direction = shift?Eigen::Vector2d(displacement/planned_distance):rear;
  recovery_shift=shift;
  reverse_yaw = yaw;
  reverse_distance = planned_distance;
  reverse_best_progress = 0.0;
  reverse_started_at = now;
  reverse_progress_at = now;
  reverse_active = true;
  action_requested_at = now;
  action_enabled = action_started = false;
  publishActionState(stamp, "accepted");
  // The previous spline must not resume after this short recovery manoeuvre.
  receive_traj = false;
  execution_failure_sent = true;
  publishExecutionFrozen(false);
  publishStop();
}

void turnRequestCallback(const std_msgs::msg::String::ConstSharedPtr &msg)
{
  if (!msg) return;
  std::istringstream input(msg->data);
  std::uint64_t stamp = 0;
  char separator = 0;
  double angle = 0.0;
  if (!(input >> stamp >> separator >> angle) || separator != '|' ||
      stamp == 0 || !std::isfinite(angle)) return;
  input >> std::ws;
  if (!input.eof() || stamp <= last_action_stamp ||
      (have_goal_time && stamp < static_cast<std::uint64_t>(latest_goal_time.value.nanoseconds()))) return;
  last_action_stamp = stamp;
  if (reverse_active) finishReverse("reverse_preempted", false);
  if (turn_active) finishTurn("turn_preempted", false);
  stop_owner=StopOwner::none;
  turn_stamp = stamp;
  turn_active = true;
  receive_traj = false;
  execution_failure_sent = true;
  publishStop();
  const auto now = legbot::Time::now();
  if (!have_odom || (now-last_odom_time).toSec()>odom_timeout ||
      std::abs(angle)<0.01 || std::abs(angle)>2.0*M_PI) {
    finishTurn("turn_request_invalid", false);
    return;
  }
  turn_start = odom_pos.head<2>();
  turn_angle = angle;
  turn_progress = 0.0;
  action_requested_at = now;
  action_enabled = action_started = false;
  publishActionState(stamp, "accepted");
}

void actionEnabledCallback(const std_msgs::msg::String::ConstSharedPtr &msg)
{
  if (!msg) return;
  const auto stamp = turn_active ? turn_stamp : (reverse_active ? reverse_stamp : 0);
  if (stamp == 0 || msg->data != std::to_string(stamp)) return;
  action_enabled = true;
  action_enable_at = legbot::Time::now();
}

void actionCancelCallback(const std_msgs::msg::String::ConstSharedPtr &msg)
{
  if (!msg) return;
  if (turn_active && msg->data == std::to_string(turn_stamp))
    finishTurn("turn_cancelled", false);
  if (reverse_active && msg->data == std::to_string(reverse_stamp))
    finishReverse("reverse_cancelled", false);
}

// A coordinator must acknowledge this exact action after activating the gait.
// No motion is published while it is absent or still issuing the mode pulse.
bool recoveryEnabled()
{
  const auto now = legbot::Time::now();
  const auto stamp = turn_active ? turn_stamp : reverse_stamp;
  if (!action_enabled) {
    publishStop();
    publishActionState(stamp, "accepted");
    if ((now-action_requested_at).toSec()>4.0) {
      if (turn_active) finishTurn("turn_activation_timeout", false);
      else finishReverse("reverse_activation_timeout", false);
    }
    return false;
  }
  if ((now-action_enable_at).toSec()>0.5) {
    if (turn_active) finishTurn("turn_activation_lost", false);
    else finishReverse("reverse_activation_lost", false);
    return false;
  }
  if (!action_started) {
    action_started = true;
    turn_started_at = reverse_started_at = reverse_progress_at = now;
    // Displacement/yaw acquired before gait activation is never progress.
    turn_progress = 0.0;
  }
  return true;
}

void acceptGoal(const scan_planner::msg::GoalRequest &request)
{
  const auto msg=std::make_shared<geometry_msgs::msg::PoseStamped>(request.target);
  const legbot::Time incoming(msg->header.stamp);
  if(have_goal_time && !incoming.isZero() && incoming.value.nanoseconds() <= latest_goal_time.value.nanoseconds())return;
  if ((turn_active || reverse_active) && !incoming.isZero() &&
      incoming.value.nanoseconds() < static_cast<std::int64_t>(last_action_stamp)) return;
  if (reverse_active) finishReverse("reverse_preempted", false);
  if (turn_active) finishTurn("turn_preempted", false);

  latest_goal_time = legbot::Time(msg->header.stamp);
  if (latest_goal_time.isZero())
    latest_goal_time = legbot::Time::now();
  have_goal_time = true;
  latest_goal_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  have_goal_position = latest_goal_pos.allFinite();
  stop_owner=StopOwner::none;
  goal_stop_latched = false;
  goal_ack_sent = false;
  execution_failure_sent = false;
  navigation_started = false;
  goal_accepted_at = std::chrono::steady_clock::now();
  navigation_phase = NavigationPhase::waiting;
  latest_goal_msg = *msg;
  current_goal_request=request;
  if(!scan_planner::validGoalReference(request)) {
    receive_traj=false;have_goal_position=false;
    failNavigation("invalid_goal_reference");return;
  }
  if(!scan_planner::validGoalTolerance(request)) {
    receive_traj=false;have_goal_position=false;
    tolerance_status_pub.publish(std_msgs::msg::String().set__data(
        scan_planner::goalToleranceStatus(request,"executor","rejected",0)));
    failNavigation("invalid_goal_tolerance");return;
  }
  finish_dist=request.has_xy_tolerance?request.xy_tolerance:default_finish_dist;
  tolerance_status_pub.publish(std_msgs::msg::String().set__data(
      scan_planner::goalToleranceStatus(request,"executor","accepted",finish_dist)));
  // A new point can be almost exactly behind the robot. Do not carry the
  // preceding goal's rotate hysteresis/direction into that U-turn.
  rotate_only = false;
  rotate_direction = 0;

  // A queued or RViz goal starts a new point-to-point mission.  Never keep
  // driving the preceding B-spline while SCAN is computing (or rejecting)
  // the replacement trajectory.  This also makes an occupied/invalid click
  // fail safe: the quadruped holds position until a fresh B-spline arrives.
  if (latest_traj_start.isZero() || latest_traj_start < latest_goal_time)
  {
    receive_traj = false;
    last_nearest_time = 0.0;
    publishExecutionFrozen(false);
    publishStop();
    ROS_INFO("[scan_go2_cmd_adapter] new goal received; previous trajectory stopped.");
  }
}

void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
  if(!msg)return;
  scan_planner::msg::GoalRequest request;request.target=*msg;acceptGoal(request);
}
void goalRequestCallback(const scan_planner::msg::GoalRequest::ConstSharedPtr &msg) {
  if(msg)acceptGoal(*msg);
}

void bsplineCallback(const scan_planner::msg::Bspline::ConstSharedPtr &msg)
{
  if (!msg || msg->order <= 0 || msg->pos_pts.size() <= static_cast<size_t>(msg->order) ||
      msg->knots.size() != msg->pos_pts.size() + msg->order + 1)
  {
    ROS_ERROR_THROTTLE(1.0, "[scan_go2_cmd_adapter] Reject malformed B-spline message.");
    return;
  }

  const legbot::Time candidate_start(msg->start_time);
  if(execution_failure_sent)return;
  if (have_goal_time && (candidate_start.isZero() || candidate_start < latest_goal_time))
  {
    ROS_WARN("[scan_go2_cmd_adapter] Ignore trajectory older than the latest goal.");
    return;
  }

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    if(!std::isfinite(msg->knots[i]) || (i && msg->knots[i]<msg->knots[i-1]))return;
    knots(i) = msg->knots[i];
  }
  const double candidate_duration=knots(msg->pos_pts.size())-knots(msg->order);
  if(!std::isfinite(candidate_duration) || candidate_duration<=0.)return;
  if(candidate_start.isZero() || !std::isfinite((legbot::Time::now()-candidate_start).toSec()) ||
     (legbot::Time::now()-candidate_start).toSec() >= candidate_duration)return;

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    if (!std::isfinite(msg->pos_pts[i].x) || !std::isfinite(msg->pos_pts[i].y) ||
        !std::isfinite(msg->pos_pts[i].z))
    {
      ROS_ERROR_THROTTLE(1.0, "[scan_go2_cmd_adapter] Reject non-finite B-spline control point.");
      return;
    }
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj_duration = traj[0].getTimeSum();
  traj_id = msg->traj_id;
  latest_traj_start = candidate_start;
  receive_traj = true;
  last_nearest_time = 0.0;
  trajectory_has_motion = false;
  for (Eigen::Index i = 1; i < pos_pts.cols(); ++i)
    trajectory_has_motion |= (pos_pts.col(i).head<2>() - pos_pts.col(0).head<2>()).norm() > 1e-4;
  if (!navigation_started) {
    navigation_started = true;
    navigation_started_at = legbot::Time::now();
  }

  ROS_INFO("[scan_go2_cmd_adapter] receive traj %d duration %.3fs", traj_id, traj_duration);
}

bool insideGoalTolerance(double xy_distance)
{
  return xy_distance<=finish_dist &&
      (!use_goal_z || std::abs(latest_goal_pos.z()-odom_pos.z())<=finish_dist_z);
}

bool measuredMotionIsQuiet()
{
  return measured_linear_speed<=stop_linear_threshold && measured_angular_speed<=stop_angular_threshold;
}

void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
  const auto &p = msg->pose.pose.position;
  const auto &q = msg->pose.pose.orientation;
  const double q_norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q_norm) || q_norm < 0.5 || q_norm > 1.5 ||
      std::max({std::abs(p.x), std::abs(p.y), std::abs(p.z)}) > 10000.0)
  {
    ROS_WARN_THROTTLE(1.0, "[scan_go2_cmd_adapter] Reject invalid odometry sample.");
    return;
  }
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  const double yaw = legbot_tf::getYaw(msg->pose.pose.orientation);
  if (turn_active && action_started) {
    const double delta = normalizeAngle(yaw-odom_yaw);
    // Odometry jumps are not evidence of completing an action.
    if (std::abs(delta)>0.5) finishTurn("turn_odometry_jump", false);
    else turn_progress += delta;
  }
  const double sample_time=static_cast<double>(msg->header.stamp.sec)+msg->header.stamp.nanosec*1e-9;
  if(!motion_samples.empty() && sample_time<motion_samples.back().t)motion_samples.clear();
  if(sample_time>0 && (motion_samples.empty() || sample_time>motion_samples.back().t)) {
    motion_samples.push_back({sample_time,odom_pos,Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized()});
    while(motion_samples.size()>2 && motion_samples[1].t<=sample_time-.20)motion_samples.pop_front();
    while(motion_samples.size()>200)motion_samples.pop_front();
    const auto &first=motion_samples.front();const auto &last=motion_samples.back();
    const double dt=last.t-first.t;
    measured_linear_speed=measured_angular_speed=std::numeric_limits<double>::infinity();
    if(dt>=.16 && dt<=.5) {
      measured_velocity=(last.p-first.p)/dt;
      measured_linear_speed=measured_velocity.norm();
      Eigen::Quaterniond rotation=first.q.conjugate()*last.q;
      if(rotation.w()<0)rotation.coeffs()*=-1.;
      const Eigen::AngleAxisd change(rotation);
      measured_rotation_rate=change.axis()*change.angle()/dt;
      measured_angular_speed=measured_rotation_rate.norm();
    }
  }
  odom_yaw = yaw;
  have_odom = true;
  last_odom_time = legbot::Time::now();
}

const char *stopName() {
  return stop_owner==StopOwner::goal?"goal":(stop_owner==StopOwner::turn?"turn":"reverse");
}
void reportStop(const char *phase,bool force=false) {
  const auto now=legbot::Time::now();
  if(!force && std::string(phase)!="settled" && !stop_status_at.isZero() && (now-stop_status_at).toSec()<.1)return;
  stop_status_at=now;
  std::ostringstream out;out<<"{\"owner\":\""<<stopName()<<"\",\"phase\":\""<<phase
      <<"\",\"elapsed\":"<<(now-stop_started).toSec()<<",\"command_speed\":"<<std::hypot(last_motion_cmd.linear.x,last_motion_cmd.linear.y)
      <<",\"command_yaw_speed\":"<<last_motion_cmd.angular.z
      <<",\"command_velocity_xyz\":["<<last_motion_cmd.linear.x<<','<<last_motion_cmd.linear.y<<','<<last_motion_cmd.linear.z<<']'
      <<",\"goal_id\":"<<current_goal_request.goal_id<<",\"attempt_id\":"<<current_goal_request.attempt_id
      <<",\"action_stamp\":"<<(stop_owner==StopOwner::turn?turn_stamp:stop_owner==StopOwner::reverse?reverse_stamp:0)
      <<",\"xy_distance\":"<<(latest_goal_pos.head<2>()-odom_pos.head<2>()).norm()
      <<",\"xy_tolerance\":"<<finish_dist
      <<",\"actual_position\":["<<odom_pos.x()<<','<<odom_pos.y()<<','<<odom_pos.z()<<']'
      <<",\"stable_duration\":"<<(stop_stable_since.isZero()?0.:(now-stop_stable_since).toSec())
      <<",\"required_stable_duration\":"<<stop_stable_seconds
      <<",\"linear_threshold\":"<<stop_linear_threshold<<",\"angular_threshold\":"<<stop_angular_threshold
      <<",\"stop_timeout\":"<<stop_timeout;
  if(std::isfinite(measured_linear_speed))out<<",\"actual_speed\":"<<measured_linear_speed
      <<",\"actual_velocity_xyz\":["<<measured_velocity.x()<<','<<measured_velocity.y()<<','<<measured_velocity.z()<<']';
  if(std::isfinite(measured_angular_speed))out<<",\"actual_angular_speed\":"<<measured_angular_speed
      <<",\"actual_rotation_rate_xyz\":["<<measured_rotation_rate.x()<<','<<measured_rotation_rate.y()<<','<<measured_rotation_rate.z()<<']';
  out<<'}';std_msgs::msg::String msg;msg.data=out.str();stop_status_pub.publish(msg);
  ROS_INFO("[scan_go2_cmd_adapter] controlled_stop %s",msg.data.c_str());
}
void beginControlledStop(StopOwner owner) {
  if(stop_owner==owner)return;
  stop_owner=owner;stop_started=legbot::Time::now();stop_stable_since=stop_status_at=legbot::Time();
  reportStop("started",true);
}
void processControlledStop() {
  const auto now=legbot::Time::now();
  if(!have_odom || (now-last_odom_time).toSec()>odom_timeout) {
    reportStop("odometry_stale",true);
    if(stop_owner==StopOwner::goal)failNavigation("execution_stop_odometry_stale");
    else if(stop_owner==StopOwner::turn)finishTurn("turn_odometry_stale",false);
    else finishReverse("reverse_odometry_stale",false);
    return;
  }
  if(stop_owner==StopOwner::goal && navigation_started &&
      (now-navigation_started_at).toSec()>=execution_timeout) {
    failNavigation("execution_timeout");return;
  }
  publishExecutionFrozen(false);
  publishSmooth(geometry_msgs::msg::Twist(),stop_deceleration,stop_yaw_deceleration);
  const bool quiet=std::hypot(last_motion_cmd.linear.x,last_motion_cmd.linear.y)<.001 &&
      std::abs(last_motion_cmd.angular.z)<.001 && measuredMotionIsQuiet();
  if(!quiet)stop_stable_since=legbot::Time();
  else if(stop_stable_since.isZero())stop_stable_since=now;
  if(quiet && (now-stop_stable_since).toSec()>=stop_stable_seconds) {
    reportStop("settled");
    const auto owner=stop_owner;stop_owner=StopOwner::none;
    if(owner==StopOwner::turn){finishTurn("succeeded",true);return;}
    if(owner==StopOwner::reverse){
      if(recovery_shift &&
         (odom_pos.head<2>()-(reverse_start+reverse_direction*reverse_distance)).norm()>.08) {
        finishReverse("shift_stop_outside_tolerance",false);return;
      }
      finishReverse("succeeded",true);return;
    }
    const double distance=(latest_goal_pos.head<2>()-odom_pos.head<2>()).norm();
    if(distance>finish_dist+1e-6 || (use_goal_z && std::abs(latest_goal_pos.z()-odom_pos.z())>finish_dist_z)) {
      failNavigation("execution_stop_outside_tolerance");return;
    }
    if(!goal_ack_sent) {
      auto ack=latest_goal_msg;ack.pose.position.x=odom_pos.x();ack.pose.position.y=odom_pos.y();ack.pose.position.z=odom_pos.z();
      goal_stop_ack_pub.publish(ack);
      tolerance_status_pub.publish(std_msgs::msg::String().set__data(
          scan_planner::goalToleranceStatus(current_goal_request,"executor","arrived",finish_dist,odom_pos.x(),odom_pos.y(),odom_pos.z(),distance)));
      goal_ack_sent=true;
    }
    return;
  }
  reportStop(quiet?"stabilizing":"braking");
  if((now-stop_started).toSec()>stop_timeout) {
    reportStop("timeout",true);
    if(stop_owner==StopOwner::turn)finishTurn("turn_stop_timeout",false);
    else if(stop_owner==StopOwner::reverse)finishReverse("reverse_stop_timeout",false);
    else failNavigation("execution_stop_timeout");
  }
}

void cmdCallback(const legbot::TimerEvent &)
{
  if ((turn_active || reverse_active) && !recoveryEnabled()) return;
  if(stop_owner!=StopOwner::none){processControlledStop();return;}
  if (turn_active) {
    const auto now = legbot::Time::now();
    if (!have_odom || (now-last_odom_time).toSec()>odom_timeout) {
      finishTurn("turn_odometry_stale", false); return;
    }
    if ((odom_pos.head<2>()-turn_start).norm()>0.25) {
      finishTurn("turn_position_deviation", false); return;
    }
    if (std::copysign(1.0,turn_angle)*turn_progress >= std::abs(turn_angle)-std::min(0.02,0.1*std::abs(turn_angle))) {
      beginControlledStop(StopOwner::turn);processControlledStop();return;
    }
    if ((now-turn_started_at).toSec()>turn_timeout) {
      finishTurn("turn_timeout", false); return;
    }
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::copysign(std::min(max_vyaw,turn_speed),turn_angle);
    publishExecutionFrozen(true);
    publishSmooth(cmd);
    return;
  }
  if (reverse_active)
  {
    const legbot::Time now = legbot::Time::now();
    if (!have_odom || (now - last_odom_time).toSec() > odom_timeout)
    {
      finishReverse("reverse_odometry_stale", false);
      return;
    }
    const Eigen::Vector2d travelled = odom_pos.head<2>() - reverse_start;
    const double progress = travelled.dot(reverse_direction);
    const Eigen::Vector2d lateral_error=travelled-progress*reverse_direction;
    const double yaw_error = normalizeAngle(reverse_yaw - odom_yaw);
    if (lateral_error.norm() > (recovery_shift?.06:.15) || std::abs(yaw_error) > 0.30 ||
        progress < (recovery_shift?-.03:-.08))
    {
      finishReverse(recovery_shift?"shift_pose_deviation":"reverse_pose_deviation", false);
      return;
    }
    if (progress >= reverse_distance - (recovery_shift?shift_stop_lead:.04))
    {
      beginControlledStop(StopOwner::reverse);processControlledStop();return;
    }
    if (progress > reverse_best_progress + 0.01)
    {
      reverse_best_progress = progress;
      reverse_progress_at = now;
    }
    if ((now - reverse_progress_at).toSec() > 3.0 ||
        (now - reverse_started_at).toSec() > 8.0)
    {
      finishReverse(recovery_shift?"shift_no_progress":"reverse_no_progress", false);
      return;
    }
    geometry_msgs::msg::Twist cmd;
    if(recovery_shift) {
      const auto velocity=clampNorm(shift_speed*reverse_direction-lateral_error,shift_speed);
      const double c=std::cos(odom_yaw),s=std::sin(odom_yaw);
      cmd.linear.x=clamp(c*velocity.x()+s*velocity.y(),-max_vx,max_vx);
      cmd.linear.y=clamp(-s*velocity.x()+c*velocity.y(),-max_vy,max_vy);
    } else cmd.linear.x = -std::min(max_vx, reverse_speed);
    cmd.angular.z = clamp(kp_yaw * yaw_error, -0.20, 0.20);
    publishExecutionFrozen(false);
    publishSmooth(cmd);
    return;
  }
  if (execution_failure_sent)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }
  if(have_goal_position && !navigation_started &&
     std::chrono::duration<double>(std::chrono::steady_clock::now()-goal_accepted_at).count() >= first_trajectory_timeout &&
     !(have_odom && (legbot::Time::now()-last_odom_time).toSec()<=odom_timeout &&
       insideGoalTolerance((latest_goal_pos.head<2>()-odom_pos.head<2>()).norm()))) {
    failNavigation("planning_wait_timeout");
    return;
  }
  if (!have_odom ||
      (legbot::Time::now() - last_odom_time).toSec() > odom_timeout)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  // The planner publishes short, repeatedly replaced local splines.  Their
  // endpoint/time is not a reliable indication that the mission goal has
  // been reached.  Latch the stop against the actual /scan/goal instead, so
  // locomotion inertia cannot carry the robot across the tolerance and make
  // the position controller command an equally large reverse step forever.
  double goal_xy_distance = std::numeric_limits<double>::infinity();
  if (have_goal_position)
  {
    goal_xy_distance = (latest_goal_pos.head<2>() - odom_pos.head<2>()).norm();
    // XY decides arrival. Optional Z is a broad same-floor gate, not a
    // precision height target.
    const bool inside_goal=insideGoalTolerance(goal_xy_distance);
    if (inside_goal && !goal_stop_latched)
      ROS_INFO("[scan_go2_cmd_adapter] arrival tolerance entered; controlled stop and physical settling before acknowledgment.");
    goal_stop_latched = goal_stop_latched || inside_goal;
    if (goal_stop_latched)
    {
      if(goal_ack_sent){publishStop();return;}
      beginControlledStop(StopOwner::goal);processControlledStop();return;
    }
  }

  if (navigation_started && have_goal_position &&
      (legbot::Time::now() - navigation_started_at).toSec() >= execution_timeout) {
    failNavigation("execution_timeout");
    return;
  }

  // A goal clicked inside the arrival circle needs no new B-spline, but it
  // still needs the stop acknowledgment above so the RL mode can end.
  if (!receive_traj)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  const double t_near = findNearestTime(traj[0], odom_pos, last_nearest_time, traj_duration);
  last_nearest_time = std::max(last_nearest_time, t_near);
  const double t_look = std::min(traj_duration, t_near + time_forward);
  const Eigen::Vector3d pos_near = traj[0].evaluateDeBoorT(t_near);
  const Eigen::Vector3d pos_look = traj[0].evaluateDeBoorT(t_look);
  const Eigen::Vector3d vel_look = traj[1].evaluateDeBoorT(t_look);

  const bool near_traj_end = t_near >= traj_duration - 0.05;
  const Eigen::Vector3d end_pt = traj[0].evaluateDeBoorT(traj_duration);
  const bool final_local_endpoint=have_goal_position && (end_pt.head<2>()-latest_goal_pos.head<2>()).norm()<.01;
  const bool reached_end =
      (end_pt.head<2>() - odom_pos.head<2>()).norm() <= finish_dist &&
      (!use_goal_z || std::abs(end_pt.z() - odom_pos.z()) <= finish_dist_z);

  const bool terminal_approach =
      have_goal_position && goal_xy_distance <= std::max(finish_dist, goal_slowdown_distance);
  if ((near_traj_end && reached_end && !final_local_endpoint) || !trajectory_has_motion)
  {
    // Reaching a short local spline is not arrival at the stamped final goal.
    // Hold for a replacement trajectory; the same watchdog bounds this wait.
    if (!navigationMakingProgress(false, terminal_approach, goal_xy_distance, 0.0)) return;
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  Eigen::Vector2d pos_err = pos_look.head<2>() - odom_pos.head<2>();
  Eigen::Vector2d vel_ff = vel_look.head<2>();
  Eigen::Vector2d vel_world = clampNorm(vel_ff + kp_pos * pos_err, max_vx);

  const double ds = (pos_look.head<2>() - pos_near.head<2>()).norm();
  if (ds > 1e-4)
  {
    const double slope = (pos_look.z() - pos_near.z()) / ds;
    const bool on_stairs = slope > slope_up_threshold || slope < slope_down_threshold;
    if (on_stairs)
    {
      vel_world *= stair_speed_scale;
      // The learned locomotion controller needs enough commanded speed to
      // step onto a riser.  A very short/replanned B-spline can otherwise
      // decay to a small non-zero command: the robot stays against the first
      // riser forever while the planner believes it is still executing.
      const double stair_speed = vel_world.norm();
      const double effective_min_stair_speed = std::min(min_stair_speed, max_vx);
      if (effective_min_stair_speed > 0.0 && stair_speed > 0.02 &&
          stair_speed < effective_min_stair_speed)
        vel_world *= effective_min_stair_speed / stair_speed;
    }
  }

  // Stair minimum-speed handling must never defeat the caller's total speed
  // limit.  Previously a 0.10 m/s experiment could produce a 0.25 m/s
  // lateral command after the minimum-speed rescale.
  vel_world = clampNorm(vel_world, max_vx);

  double yaw_des = estimateDesiredYaw(t_near, t_look, pos_near, pos_look);
  // Position-error steering is useful only during coarse tracking.  Near the
  // final point its direction flips by pi as soon as the robot crosses the
  // target, producing the observed left/right oscillation.  Always retain
  // the stable spline-tangent heading in the terminal approach zone.
  if (steer_with_position_error && !terminal_approach && vel_world.norm() > 0.02)
    yaw_des = std::atan2(vel_world(1), vel_world(0));
  const double yaw_err = normalizeAngle(yaw_des - odom_yaw);

  const double abs_yaw_error = std::abs(yaw_err);
  // Preserve the existing coarse-heading policy outside the terminal zone;
  // close to the goal use tighter hysteresis instead of disabling rotation.
  const double rotate_enter = terminal_approach ? terminal_rotate_enter : heading_error_threshold;
  const double rotate_exit = terminal_approach ? terminal_rotate_exit :
      heading_error_threshold * rotate_exit_ratio;
  if (!rotate_only && abs_yaw_error > rotate_enter)
  {
    rotate_only = true;
    rotate_direction = yaw_err >= 0.0 ? 1 : -1;
    ROS_INFO("[scan_go2_cmd_adapter] Enter rotate-in-place: yaw_error=%.2f rad.", yaw_err);
  }
  else if (rotate_only && abs_yaw_error <= rotate_exit)
  {
    rotate_only = false;
    rotate_direction = 0;
    ROS_INFO("[scan_go2_cmd_adapter] Exit rotate-in-place: yaw_error=%.2f rad.", yaw_err);
  }
  // Latch the direction only near the +/-pi wrap. A changed spline must not
  // make a small correction continue in the wrong direction.
  if (rotate_only && abs_yaw_error < M_PI / 2.0)
    rotate_direction = yaw_err >= 0.0 ? 1 : -1;

  if (!navigationMakingProgress(rotate_only, terminal_approach,
                                goal_xy_distance, abs_yaw_error)) return;

  const double controlled_yaw_error =
      rotate_only && rotate_direction != 0
          ? static_cast<double>(rotate_direction) * abs_yaw_error
          : yaw_err;
  const double vyaw_cmd = clamp(
      kp_yaw * controlled_yaw_error, -max_vyaw, max_vyaw);

  // Separate enter/exit thresholds prevent the controller from alternating
  // between rotate-in-place and translation at the same angular boundary.
  if (rotate_only)
  {
    publishExecutionFrozen(true);
    // Ramp angular velocity just like translation. The old direct publish
    // jumped between 0 and max yaw rate, which made GO2's learned gait look
    // stuck/jerky at waypoint reversals.
    geometry_msgs::msg::Twist rotate_cmd;
    rotate_cmd.angular.z = vyaw_cmd;
    publishSmooth(rotate_cmd);
    ROS_INFO_THROTTLE(1.0,
                      "[scan_go2_cmd_adapter] Rotating: current=%.2f desired=%.2f "
                      "error=%.2f cmd=%.2f rad/s.",
                      odom_yaw, yaw_des, yaw_err, vyaw_cmd);
    return;
  }

  publishExecutionFrozen(false);

  // A quadruped loses stability when a large translational command is mixed
  // with a sharp heading correction. Reduce translation continuously before
  // the rotate-in-place threshold rather than switching abruptly at it.
  if (heading_error_threshold > turn_slowdown_angle &&
      abs_yaw_error > turn_slowdown_angle)
  {
    const double ratio = clamp(
        (heading_error_threshold - abs_yaw_error) /
            (heading_error_threshold - turn_slowdown_angle),
        0.0, 1.0);
    const double scale = min_turn_speed_scale + (1.0 - min_turn_speed_scale) * ratio;
    vel_world *= scale;
  }

  // Keep tracking the spline. Floor only a forward command on a live
  // segment while still outside the goal's horizontal arrival tolerance.
  const double walk_floor = std::min(min_walk_speed, max_vx);
  const double walk_speed = vel_world.norm();
  if (have_goal_position && goal_xy_distance > finish_dist &&
      abs_yaw_error <= turn_slowdown_angle && walk_speed > 1e-6 && walk_speed < walk_floor)
  {
    Eigen::Vector2d forward = (pos_look - pos_near).head<2>();
    if (forward.squaredNorm() < 1e-8)
      forward = (pos_near - traj[0].evaluateDeBoorT(std::max(0.0, t_near - time_forward))).head<2>();
    if (forward.squaredNorm() >= 1e-8 && vel_world.dot(forward) > 0.0)
      vel_world *= walk_floor / walk_speed;
  }

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;
  publishSmooth(cmd);
}
} // namespace

int main(int argc, char **argv)
{
  legbot::init(argc, argv, "scan_go2_cmd_adapter");
  legbot::NodeHandle node;
  legbot::NodeHandle nh("~");

  loadParams(nh);

  bspline_sub = node.subscribe("/scan/planning/bspline", 10, bsplineCallback,
                               legbot::TransportHints().tcpNoDelay());
  goal_sub = node.subscribe("/scan/goal", 10, goalCallback,
                            legbot::TransportHints().tcpNoDelay());
  goal_request_sub = node.subscribe("/scan/goal_request",10,goalRequestCallback,
                                     legbot::TransportHints().tcpNoDelay());
  turn_request_sub = node.subscribe("/scan/recovery_turn_request", 10,
                                    turnRequestCallback,
                                    legbot::TransportHints().tcpNoDelay());
  action_cancel_sub = node.subscribe("/scan/recovery_action_cancel", 10,
                                     actionCancelCallback,
                                     legbot::TransportHints().tcpNoDelay());
  action_enabled_sub = node.subscribe("/scan/recovery_action_enabled", 10,
                                      actionEnabledCallback,
                                      legbot::TransportHints().tcpNoDelay());
  reverse_request_sub = node.subscribe("/scan/recovery_reverse_request", 10,
                                       reverseRequestCallback,
                                       legbot::TransportHints().tcpNoDelay());
  odom_sub = node.subscribe(body_pose_topic, 20, odomCallback,
                            legbot::TransportHints().tcpNoDelay());
  cmd_vel_pub = node.advertise<geometry_msgs::msg::Twist>("/cmd_vel", 20);
  execution_frozen_pub = node.advertise<std_msgs::msg::Bool>("/scan/planning/execution_frozen", 10);
  stop_status_pub=node.advertise<std_msgs::msg::String>("/scan/execution_stop_status",10);
  goal_stop_ack_pub = node.advertise<geometry_msgs::msg::PoseStamped>("/scan/goal_stop_ack", 10);
  goal_failure_pub = node.advertise<std_msgs::msg::String>("/scan/goal_plan_failed", 10);
  tolerance_status_pub = node.advertise<std_msgs::msg::String>("/scan/goal_tolerance_status",10);
  reverse_result_pub = node.advertise<std_msgs::msg::String>(
      "/scan/recovery_reverse_result", 10);
  turn_result_pub = node.advertise<std_msgs::msg::String>(
      "/scan/recovery_turn_result", 10);
  recovery_state_pub = node.advertise<std_msgs::msg::String>(
      "/scan/recovery_action_state", 10);
  cmd_timer = node.createTimer(legbot::Duration(0.01), cmdCallback);

  ROS_INFO("[scan_go2_cmd_adapter] ready. bspline=/scan/planning/bspline odom=%s",
           body_pose_topic.c_str());

  legbot::spin();

  cmd_timer.stop();
  bspline_sub.shutdown();
  goal_sub.shutdown();
  goal_request_sub.shutdown();
  turn_request_sub.shutdown();
  action_cancel_sub.shutdown();
  action_enabled_sub.shutdown();
  reverse_request_sub.shutdown();
  odom_sub.shutdown();
  cmd_vel_pub.shutdown();
  execution_frozen_pub.shutdown();
  stop_status_pub.shutdown();
  goal_stop_ack_pub.shutdown();
  goal_failure_pub.shutdown();
  tolerance_status_pub.shutdown();
  reverse_result_pub.shutdown();
  turn_result_pub.shutdown();
  recovery_state_pub.shutdown();
  return 0;
}
