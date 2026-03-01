#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <Eigen/Dense>
#include <memory>
#include <string>

/**
 * @brief Takeoff node that generates a smooth vertical trajectory
 * 
 * Uses a 5th-order polynomial (minimum-jerk trajectory) to generate
 * smooth position, velocity, and acceleration profiles for takeoff.
 */
class TakeoffNode : public rclcpp::Node
{
public:
  TakeoffNode();

private:
  // Callbacks
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publishTrajectory();

  // Trajectory generation using 5th-order polynomial (minimum-jerk)
  // s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
  // Boundary conditions: s(0)=0, s(T)=1, s'(0)=0, s'(T)=0, s''(0)=0, s''(T)=0
  // Solution: s(t) = 10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5
  double smoothStep(double t, double T);
  double smoothStepVel(double t, double T);
  double smoothStepAcc(double t, double T);

  // ROS2 entities
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Parameters
  double takeoff_height_;
  double takeoff_duration_;
  double publish_rate_;
  std::string odom_topic_;
  std::string trajectory_topic_;

  // State
  bool received_initial_pose_ = false;
  bool takeoff_complete_ = false;
  Eigen::Vector3d start_position_;
  Eigen::Vector3d target_position_;
  double start_yaw_ = 0.0;
  rclcpp::Time start_time_;
};
