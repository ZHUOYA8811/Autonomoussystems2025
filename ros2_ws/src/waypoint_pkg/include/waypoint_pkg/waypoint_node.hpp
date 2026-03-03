#ifndef WAYPOINT_PKG__WAYPOINT_NODE_HPP_
#define WAYPOINT_PKG__WAYPOINT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <Eigen/Dense>
#include <vector>

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

  // Trajectory generation
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
  Eigen::Vector4d current_orientation_;  // quaternion (w, x, y, z)
  bool received_odom_;
  bool trajectory_active_;

  // Trajectory data
  Eigen::Vector3d traj_start_;
  Eigen::Vector3d traj_goal_;
  double traj_start_time_;
  double traj_duration_;

  // Parameters
  double waypoint_threshold_;  // distance to consider waypoint reached
  double cruise_speed_;        // m/s
  double publish_rate_;
  double takeoff_height_;
};

}  // namespace waypoint_pkg

#endif  // WAYPOINT_PKG__WAYPOINT_NODE_HPP_
