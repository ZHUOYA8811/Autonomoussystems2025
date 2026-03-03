#ifndef SIMPLE_TRAJECTORY_PLANNER_HPP
#define SIMPLE_TRAJECTORY_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>
#include <state_machine/msg/command.hpp>
#include <state_machine/msg/answer.hpp>

#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/ros_conversions.h>

class SimpleTrajectoryPlanner : public rclcpp::Node {
public:
    SimpleTrajectoryPlanner();
    ~SimpleTrajectoryPlanner() = default;

private:
    // Callbacks
    void frontierPointCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void currentStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void commandCallback(const state_machine::msg::Command::SharedPtr msg);
    void healthReportCallback();

    // Planning functions
    void planTrajectory(const Eigen::Vector3d& goal_position);
    bool generatePolynomialTrajectory(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& start_vel,
        const Eigen::Vector3d& goal_pos,
        mav_trajectory_generation::Trajectory& trajectory);

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_frontier_point_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_current_state_;
    rclcpp::Subscription<state_machine::msg::Command>::SharedPtr sub_command_;

    rclcpp::Publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr pub_trajectory_;
    rclcpp::Publisher<state_machine::msg::Answer>::SharedPtr pub_node_health_;

    rclcpp::TimerBase::SharedPtr timer_health_report_;

    // State
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    bool has_current_state_{false};
    bool is_active_{false};

    // Parameters
    double max_velocity_{3.0};
    double max_acceleration_{2.0};
    double health_report_period_{1.0};
};

#endif // SIMPLE_TRAJECTORY_PLANNER_HPP
