#include "waypoint_pkg/waypoint_node.hpp"
#include <cmath>

namespace waypoint_pkg
{

WaypointNode::WaypointNode()
: Node("waypoint_node"),
  current_waypoint_idx_(0),
  received_odom_(false),
  trajectory_active_(false),
  traj_start_time_(0.0),
  traj_duration_(5.0)
{
  // Declare parameters
  this->declare_parameter("waypoint_threshold", 0.5);
  this->declare_parameter("cruise_speed", 2.0);
  this->declare_parameter("publish_rate", 100.0);
  this->declare_parameter("takeoff_height", 5.0);
  this->declare_parameter("odom_topic", "current_state_est");
  this->declare_parameter("trajectory_topic", "command/trajectory");

  // Get parameters
  waypoint_threshold_ = this->get_parameter("waypoint_threshold").as_double();
  cruise_speed_ = this->get_parameter("cruise_speed").as_double();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  takeoff_height_ = this->get_parameter("takeoff_height").as_double();
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
  if (current_waypoint_idx_ < waypoints_.size()) {
    double dist = (current_position_ - waypoints_[current_waypoint_idx_]).norm();
    
    if (dist < waypoint_threshold_) {
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
  traj_start_ = start;
  traj_goal_ = goal;
  
  // 计算轨迹时间（基于距离和巡航速度）
  double distance = (goal - start).norm();
  traj_duration_ = std::max(distance / cruise_speed_, 1.0);  // 至少1秒
  
  traj_start_time_ = this->now().seconds();
  trajectory_active_ = true;

  RCLCPP_INFO(this->get_logger(), 
    "Generating trajectory: (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f), duration=%.2fs",
    start.x(), start.y(), start.z(),
    goal.x(), goal.y(), goal.z(),
    traj_duration_);
}

void WaypointNode::publishTrajectoryPoint()
{
  double current_time = this->now().seconds();
  double t = current_time - traj_start_time_;
  
  // Clamp time (允许超过duration以维持悬停commanded state)
  if (t < 0) t = 0;
  if (t > traj_duration_) t = traj_duration_;
  
  double T = traj_duration_;
  double tau = t / T;  // normalized time [0, 1]
  
  // 5th order polynomial (minimum-jerk): s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
  double tau2 = tau * tau;
  double tau3 = tau2 * tau;
  double tau4 = tau3 * tau;
  double tau5 = tau4 * tau;
  
  double s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
  double s_dot = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / T;
  double s_ddot = (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) / (T * T);
  
  // Calculate position, velocity, acceleration
  Eigen::Vector3d delta = traj_goal_ - traj_start_;
  Eigen::Vector3d pos = traj_start_ + s * delta;
  Eigen::Vector3d vel = s_dot * delta;
  Eigen::Vector3d acc = s_ddot * delta;
  
  // Build message
  trajectory_msgs::msg::MultiDOFJointTrajectory traj_msg;
  traj_msg.header.stamp = this->now();
  traj_msg.header.frame_id = "world";
  
  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
  
  // Position
  geometry_msgs::msg::Transform transform;
  transform.translation.x = pos.x();
  transform.translation.y = pos.y();
  transform.translation.z = pos.z();
  transform.rotation.w = 1.0;
  transform.rotation.x = 0.0;
  transform.rotation.y = 0.0;
  transform.rotation.z = 0.0;
  point.transforms.push_back(transform);
  
  // Velocity
  geometry_msgs::msg::Twist twist;
  twist.linear.x = vel.x();
  twist.linear.y = vel.y();
  twist.linear.z = vel.z();
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
  point.velocities.push_back(twist);
  
  // Acceleration
  geometry_msgs::msg::Twist accel;
  accel.linear.x = acc.x();
  accel.linear.y = acc.y();
  accel.linear.z = acc.z();
  accel.angular.x = 0.0;
  accel.angular.y = 0.0;
  accel.angular.z = 0.0;
  point.accelerations.push_back(accel);
  
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
