#include "takeoff_pkg/takeoff_node.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

TakeoffNode::TakeoffNode()
: rclcpp::Node("takeoff_node")
{
  // Declare and get parameters
  this->declare_parameter<double>("takeoff_height", 5.0);
  this->declare_parameter<double>("takeoff_duration", 5.0);
  this->declare_parameter<double>("publish_rate", 100.0);
  this->declare_parameter<std::string>("odom_topic", "current_state");
  this->declare_parameter<std::string>("trajectory_topic", "command/trajectory");

  takeoff_height_ = this->get_parameter("takeoff_height").as_double();
  takeoff_duration_ = this->get_parameter("takeoff_duration").as_double();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  trajectory_topic_ = this->get_parameter("trajectory_topic").as_string();

  RCLCPP_INFO(this->get_logger(), 
    "Takeoff node initialized: height=%.1fm, duration=%.1fs, rate=%.0fHz",
    takeoff_height_, takeoff_duration_, publish_rate_);

  // Create subscriber for odometry
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(10),
    std::bind(&TakeoffNode::onOdometry, this, std::placeholders::_1));

  // Create publisher for trajectory commands
  traj_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
    trajectory_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "Waiting for initial odometry on topic '%s'...", odom_topic_.c_str());
}

void TakeoffNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!received_initial_pose_) {
    // Store initial position
    start_position_ << msg->pose.pose.position.x,
                       msg->pose.pose.position.y,
                       msg->pose.pose.position.z;
    
    // Calculate target position (vertical takeoff)
    target_position_ = start_position_;
    target_position_.z() += takeoff_height_;

    // Extract yaw from quaternion
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    start_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 
                            1.0 - 2.0 * (qy * qy + qz * qz));

    received_initial_pose_ = true;
    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), 
      "Initial pose received: [%.2f, %.2f, %.2f], yaw=%.2f rad",
      start_position_.x(), start_position_.y(), start_position_.z(), start_yaw_);
    RCLCPP_INFO(this->get_logger(), 
      "Takeoff target: [%.2f, %.2f, %.2f]",
      target_position_.x(), target_position_.y(), target_position_.z());

    // Start trajectory publishing timer
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TakeoffNode::publishTrajectory, this));
  }
}

void TakeoffNode::publishTrajectory()
{
  if (takeoff_complete_) {
    return;
  }

  // Calculate elapsed time
  double t = (this->now() - start_time_).seconds();
  
  if (t > takeoff_duration_) {
    t = takeoff_duration_;
    takeoff_complete_ = true;
    RCLCPP_INFO(this->get_logger(), "Takeoff trajectory complete, hovering at target.");
  }

  // Calculate smooth trajectory using minimum-jerk polynomial
  double s = smoothStep(t, takeoff_duration_);
  double s_dot = smoothStepVel(t, takeoff_duration_);
  double s_ddot = smoothStepAcc(t, takeoff_duration_);

  // Position: linear interpolation with smooth s
  Eigen::Vector3d pos = start_position_ + s * (target_position_ - start_position_);
  
  // Velocity: derivative of position
  Eigen::Vector3d vel = s_dot * (target_position_ - start_position_);
  
  // Acceleration: second derivative of position
  Eigen::Vector3d acc = s_ddot * (target_position_ - start_position_);

  // Generate yaw quaternion (constant yaw during takeoff)
  double cy = std::cos(start_yaw_ * 0.5);
  double sy = std::sin(start_yaw_ * 0.5);

  // Build trajectory message
  trajectory_msgs::msg::MultiDOFJointTrajectory traj_msg;
  traj_msg.header.stamp = this->now();
  traj_msg.header.frame_id = "world";

  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
  
  // Transform (position + orientation)
  geometry_msgs::msg::Transform transform;
  transform.translation.x = pos.x();
  transform.translation.y = pos.y();
  transform.translation.z = pos.z();
  transform.rotation.x = 0.0;
  transform.rotation.y = 0.0;
  transform.rotation.z = sy;
  transform.rotation.w = cy;
  point.transforms.push_back(transform);

  // Velocity
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = vel.x();
  velocity.linear.y = vel.y();
  velocity.linear.z = vel.z();
  velocity.angular.x = 0.0;
  velocity.angular.y = 0.0;
  velocity.angular.z = 0.0;
  point.velocities.push_back(velocity);

  // Acceleration
  geometry_msgs::msg::Twist acceleration;
  acceleration.linear.x = acc.x();
  acceleration.linear.y = acc.y();
  acceleration.linear.z = acc.z();
  acceleration.angular.x = 0.0;
  acceleration.angular.y = 0.0;
  acceleration.angular.z = 0.0;
  point.accelerations.push_back(acceleration);

  // Time from start
  point.time_from_start.sec = static_cast<int32_t>(t);
  point.time_from_start.nanosec = static_cast<uint32_t>((t - point.time_from_start.sec) * 1e9);

  traj_msg.points.push_back(point);

  // Publish
  traj_pub_->publish(traj_msg);
}

// 5th-order polynomial for smooth trajectory: s(t) = 10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5
// This satisfies: s(0)=0, s(T)=1, s'(0)=0, s'(T)=0, s''(0)=0, s''(T)=0
double TakeoffNode::smoothStep(double t, double T)
{
  if (t <= 0.0) return 0.0;
  if (t >= T) return 1.0;
  
  double tau = t / T;
  double tau3 = tau * tau * tau;
  double tau4 = tau3 * tau;
  double tau5 = tau4 * tau;
  
  return 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
}

// Derivative: s'(t) = (30*tau^2 - 60*tau^3 + 30*tau^4) / T
double TakeoffNode::smoothStepVel(double t, double T)
{
  if (t <= 0.0 || t >= T) return 0.0;
  
  double tau = t / T;
  double tau2 = tau * tau;
  double tau3 = tau2 * tau;
  double tau4 = tau3 * tau;
  
  return (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / T;
}

// Second derivative: s''(t) = (60*tau - 180*tau^2 + 120*tau^3) / T^2
double TakeoffNode::smoothStepAcc(double t, double T)
{
  if (t <= 0.0 || t >= T) return 0.0;
  
  double tau = t / T;
  double tau2 = tau * tau;
  double tau3 = tau2 * tau;
  
  return (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) / (T * T);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakeoffNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
