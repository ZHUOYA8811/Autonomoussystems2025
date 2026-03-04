#ifndef WAYPOINT_PKG__WAYPOINT_NODE_HPP_
#define WAYPOINT_PKG__WAYPOINT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <Eigen/Dense>
#include <vector>

// ETH mav_trajectory_generation (linear solver – microsecond solve, safe in RT loop)
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation/vertex.h>
#include <mav_msgs/conversions.hpp>
#include <mav_msgs/eigen_mav_msgs.hpp>

namespace waypoint_pkg
{

class WaypointNode : public rclcpp::Node
{
public:
  WaypointNode();

private:
  // Callbacks
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();

  // Trajectory generation (now uses mav_trajectory_generation)
  void generateTrajectoryToWaypoint(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
  void publishTrajectoryPoint();

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publishers
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Waypoints
  std::vector<Eigen::Vector3d> waypoints_;
  size_t current_waypoint_idx_;

  // State
  Eigen::Vector3d current_position_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector4d current_orientation_;  // quaternion (w, x, y, z)
  bool received_odom_;
  bool trajectory_active_;

  // Optimised polynomial trajectory (ETH linear solver)
  mav_trajectory_generation::Trajectory trajectory_;
  Eigen::Vector3d traj_end_pos_;   // goal of current segment (for clean hover hold)
  bool   traj_valid_;
  double traj_start_time_;         // wall-clock seconds when this segment started

  // Parameters
  double waypoint_threshold_;  // distance to consider waypoint reached
  double cruise_speed_;        // m/s
  double max_acceleration_;    // m/s²
  double publish_rate_;
  double takeoff_height_;
};

}  // namespace waypoint_pkg

#endif  // WAYPOINT_PKG__WAYPOINT_NODE_HPP_
