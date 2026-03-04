#include "waypoint_pkg/waypoint_node.hpp"
#include <cmath>

namespace waypoint_pkg
{

WaypointNode::WaypointNode()
: Node("waypoint_node"),
  current_waypoint_idx_(0),
  current_position_(Eigen::Vector3d::Zero()),
  current_velocity_(Eigen::Vector3d::Zero()),
  received_odom_(false),
  trajectory_active_(false),
  traj_end_pos_(Eigen::Vector3d::Zero()),
  traj_valid_(false),
  traj_start_time_(0.0)
{
  // Declare parameters
  this->declare_parameter("waypoint_threshold", 0.5);
  this->declare_parameter("cruise_speed", 2.0);
  this->declare_parameter("max_acceleration", 2.0);
  this->declare_parameter("publish_rate", 100.0);
  this->declare_parameter("takeoff_height", 5.0);
  this->declare_parameter("odom_topic", "current_state_est");
  this->declare_parameter("trajectory_topic", "command/trajectory");

  // Get parameters
  waypoint_threshold_ = this->get_parameter("waypoint_threshold").as_double();
  cruise_speed_       = this->get_parameter("cruise_speed").as_double();
  max_acceleration_   = this->get_parameter("max_acceleration").as_double();
  publish_rate_       = this->get_parameter("publish_rate").as_double();
  takeoff_height_     = this->get_parameter("takeoff_height").as_double();
  std::string odom_topic = this->get_parameter("odom_topic").as_string();
  std::string traj_topic = this->get_parameter("trajectory_topic").as_string();

  // Initialize waypoints (起飞后的航点任务)
  // 航点0: 起飞到指定高度 (在收到第一帧后动态设置)
  // 航点1-N: 预设飞行任务
  waypoints_.push_back(Eigen::Vector3d(0.0, 0.0, takeoff_height_));  // 起飞点(会被更新)
  waypoints_.push_back(Eigen::Vector3d(5.0, 0.0, takeoff_height_));  // 前进5米
  waypoints_.push_back(Eigen::Vector3d(5.0, 5.0, takeoff_height_));  // 右转5米
  waypoints_.push_back(Eigen::Vector3d(0.0, 5.0, takeoff_height_));  // 返回
  waypoints_.push_back(Eigen::Vector3d(0.0, 0.0, takeoff_height_));  // 回到起点上空

  // Create subscriber
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    std::bind(&WaypointNode::onOdometry, this, std::placeholders::_1));

  // Create publishers
  traj_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
    traj_topic, 10);
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "current_goal", 10);

  // Create timer
  double dt = 1.0 / publish_rate_;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(dt),
    std::bind(&WaypointNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), 
    "Waypoint node initialized: %zu waypoints, speed=%.1f m/s, threshold=%.2f m",
    waypoints_.size(), cruise_speed_, waypoint_threshold_);
  RCLCPP_INFO(this->get_logger(), "Waiting for odometry on topic '%s'...", odom_topic.c_str());
}

void WaypointNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_position_ <<
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z;

  current_velocity_ <<
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z;

  current_orientation_ <<
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z;

  if (!received_odom_) {
    received_odom_ = true;
    
    // 更新第一个航点为起飞目标（当前xy + 目标高度）
    waypoints_[0] = Eigen::Vector3d(
      current_position_.x(), 
      current_position_.y(), 
      takeoff_height_);
    
    // 更新后续航点的xy偏移（相对于起始位置）
    Eigen::Vector3d start_xy(current_position_.x(), current_position_.y(), 0.0);
    for (size_t i = 1; i < waypoints_.size(); ++i) {
      waypoints_[i].x() += start_xy.x();
      waypoints_[i].y() += start_xy.y();
    }

    RCLCPP_INFO(this->get_logger(), 
      "Received first odometry at (%.2f, %.2f, %.2f). Starting mission.",
      current_position_.x(), current_position_.y(), current_position_.z());
    
    RCLCPP_INFO(this->get_logger(),
      "Waypoint 0 (takeoff): (%.2f, %.2f, %.2f)",
      waypoints_[0].x(), waypoints_[0].y(), waypoints_[0].z());
    
    // 开始第一段轨迹（起飞）
    generateTrajectoryToWaypoint(current_position_, waypoints_[0]);
  }
}

void WaypointNode::controlLoop()
{
  if (!received_odom_) {
    return;
  }

  // 发布当前目标点（用于可视化）
  geometry_msgs::msg::PointStamped goal_msg;
  goal_msg.header.stamp = this->now();
  goal_msg.header.frame_id = "world";
  if (current_waypoint_idx_ < waypoints_.size()) {
    goal_msg.point.x = waypoints_[current_waypoint_idx_].x();
    goal_msg.point.y = waypoints_[current_waypoint_idx_].y();
    goal_msg.point.z = waypoints_[current_waypoint_idx_].z();
  }
  goal_pub_->publish(goal_msg);

  // 检查是否到达当前航点
  // Guard: only evaluate arrival after at least 30 % of the segment has elapsed.
  // This prevents false triggers at the very start of a new segment (where the
  // optimised trajectory hasn't had time to pull the drone away yet) and avoids
  // calling generateTrajectoryToWaypoint with a stale / noisy velocity snapshot.
  if (current_waypoint_idx_ < waypoints_.size()) {
    const double elapsed_frac = (traj_valid_ && trajectory_.getMaxTime() > 0.0)
        ? (this->now().seconds() - traj_start_time_) / trajectory_.getMaxTime()
        : 1.0;

    double dist = (current_position_ - waypoints_[current_waypoint_idx_]).norm();

    if (elapsed_frac >= 0.30 && dist < waypoint_threshold_) {
      RCLCPP_INFO(this->get_logger(), 
        "Reached waypoint %zu at (%.2f, %.2f, %.2f)",
        current_waypoint_idx_,
        waypoints_[current_waypoint_idx_].x(),
        waypoints_[current_waypoint_idx_].y(),
        waypoints_[current_waypoint_idx_].z());
      
      current_waypoint_idx_++;
      
      if (current_waypoint_idx_ < waypoints_.size()) {
        // 生成到下一个航点的轨迹
        generateTrajectoryToWaypoint(current_position_, waypoints_[current_waypoint_idx_]);
      } else {
        RCLCPP_INFO(this->get_logger(), "All waypoints reached! Hovering at final position.");
        // 保持 trajectory_active = true，继续发布悬停命令
        // 将最后的航点作为悬停点
        generateTrajectoryToWaypoint(current_position_, current_position_);
      }
    }
  }

  // 发布轨迹点
  if (trajectory_active_) {
    publishTrajectoryPoint();
  }
}

void WaypointNode::generateTrajectoryToWaypoint(
  const Eigen::Vector3d& start,
  const Eigen::Vector3d& goal)
{
  using namespace mav_trajectory_generation;

  constexpr int D = 3;   // 3-D position
  constexpr int N = 10;  // polynomial degree (10 coefficients → degree-9)
  const int derivative_to_optimize = derivative_order::SNAP;  // minimise snap

  // ---- Vertices ----
  Vertex::Vector vertices;

  Vertex start_v(D);
  start_v.makeStartOrEnd(start, derivative_to_optimize);
  // Clamp measured velocity to [0, cruise_speed_] to keep the polynomial
  // well-conditioned even when the state estimator has a noisy spike at the
  // moment of waypoint switching.
  Eigen::Vector3d v0 = current_velocity_;
  if (v0.norm() > cruise_speed_) { v0 = v0.normalized() * cruise_speed_; }
  start_v.addConstraint(derivative_order::VELOCITY, v0);
  vertices.push_back(start_v);

  Vertex end_v(D);
  end_v.makeStartOrEnd(goal, derivative_to_optimize);
  end_v.addConstraint(derivative_order::VELOCITY,     Eigen::Vector3d::Zero());
  end_v.addConstraint(derivative_order::ACCELERATION, Eigen::Vector3d::Zero());
  vertices.push_back(end_v);

  // ---- Segment-time estimate (velocity-ramp model, respects v_max & a_max) ----
  std::vector<double> segment_times =
    estimateSegmentTimes(vertices, cruise_speed_, max_acceleration_);

  // ---- Linear solver (runs in microseconds – no iteration, safe in RT loop) ----
  PolynomialOptimization<N> opt(D);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  // ---- Store result ----
  opt.getTrajectory(&trajectory_);
  traj_end_pos_    = goal;
  traj_valid_      = true;
  traj_start_time_ = this->now().seconds();
  trajectory_active_ = true;

  RCLCPP_INFO(this->get_logger(),
    "[mav_traj/linear] (%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f)  T=%.2fs  v0=%.2fm/s",
    start.x(), start.y(), start.z(),
    goal.x(),  goal.y(),  goal.z(),
    trajectory_.getMaxTime(), v0.norm());
}

void WaypointNode::publishTrajectoryPoint()
{
  if (!traj_valid_) { return; }

  double t = this->now().seconds() - traj_start_time_;
  if (t < 0.0) { t = 0.0; }
  const double T_max = trajectory_.getMaxTime();

  trajectory_msgs::msg::MultiDOFJointTrajectory traj_msg;
  traj_msg.header.stamp    = this->now();
  traj_msg.header.frame_id = "world";
  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;

  if (t >= T_max) {
    // ---- Trajectory finished: publish a static hover at the exact goal ----
    // Explicitly zeroing vel and acc avoids numerical garbage from sampleTrajectoryAtTime
    // at the boundary, which would appear as a non-zero feed-forward force in the
    // geometric controller and cause the drone to drift / buzz.
    geometry_msgs::msg::Transform transform;
    transform.translation.x = traj_end_pos_.x();
    transform.translation.y = traj_end_pos_.y();
    transform.translation.z = traj_end_pos_.z();
    transform.rotation.w = 1.0;
    point.transforms.push_back(transform);
    point.velocities.push_back(geometry_msgs::msg::Twist{});     // zero
    point.accelerations.push_back(geometry_msgs::msg::Twist{});  // zero
    traj_msg.points.push_back(point);
    traj_pub_->publish(traj_msg);
    return;
  }

  // ---- Active trajectory: sample optimised polynomial ----
  mav_msgs::EigenTrajectoryPoint sample;
  const bool ok = mav_trajectory_generation::sampleTrajectoryAtTime(
    trajectory_, t, &sample);
  if (!ok) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[mav_traj] sampleTrajectoryAtTime failed at t=%.3f", t);
    return;
  }

  geometry_msgs::msg::Transform transform;
  transform.translation.x = sample.position_W.x();
  transform.translation.y = sample.position_W.y();
  transform.translation.z = sample.position_W.z();
  transform.rotation.w = 1.0;
  point.transforms.push_back(transform);

  geometry_msgs::msg::Twist vel_msg;
  vel_msg.linear.x = sample.velocity_W.x();
  vel_msg.linear.y = sample.velocity_W.y();
  vel_msg.linear.z = sample.velocity_W.z();
  point.velocities.push_back(vel_msg);

  geometry_msgs::msg::Twist acc_msg;
  acc_msg.linear.x = sample.acceleration_W.x();
  acc_msg.linear.y = sample.acceleration_W.y();
  acc_msg.linear.z = sample.acceleration_W.z();
  point.accelerations.push_back(acc_msg);

  traj_msg.points.push_back(point);
  traj_pub_->publish(traj_msg);
}

}  // namespace waypoint_pkg

// Main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<waypoint_pkg::WaypointNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
