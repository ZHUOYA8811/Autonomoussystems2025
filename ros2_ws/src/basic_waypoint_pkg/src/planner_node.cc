// MissionPlannerNode:
// - Receives StateMachine commands on "statemachine/cmd" (state_machine::msg::Command)
// - Only handles commands addressed to this node (msg->target == receiver_name, default "planner").
// - START without target => enter EXPLORING mode, follow planned_path and replan periodically.
// - START with target    => one-shot plan to msg->target_pos.
// - HOLD / LAND / ABORT  => exit EXPLORING mode (ignore planned_path).
//
// Output:
// - BasicPlanner publishes mav_planning_msgs::PolynomialTrajectory4D on topic "trajectory"
// - trajectory_sampler_node remaps ("path_segments_4D" -> "trajectory") and publishes controller commands

#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <eigen3/Eigen/Dense>

#include "state_machine/msg/command.hpp"
#include "basic_waypoint_pkg/planner.hpp"

class MissionPlannerNode : public rclcpp::Node {
public:
  explicit MissionPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("basic_waypoint_node", options)
  {
    // Match StateMachine's cmd_msg.target for planner commands
    receiver_name_ = this->declare_parameter<std::string>("receiver_name", "planner");

    // Topics
    cmd_topic_  = this->declare_parameter<std::string>("command_topic", "statemachine/cmd");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "planned_path");

    // Replan throttling (avoid resetting sampler too often)
    min_replan_period_sec_  = this->declare_parameter<double>("min_replan_period_sec", 0.5);
    min_goal_change_dist_m_ = this->declare_parameter<double>("min_goal_change_dist_m", 0.5);

    RCLCPP_INFO(
      get_logger(),
      "MissionPlannerNode configured: receiver='%s', cmd_topic='%s', path_topic='%s'",
      receiver_name_.c_str(), cmd_topic_.c_str(), path_topic_.c_str());
  }

  // Call after node
  void init(const rclcpp::Node::SharedPtr& self_shared)
  {
    planner_ = std::make_unique<BasicPlanner>(self_shared);

    sub_cmd_ = this->create_subscription<state_machine::msg::Command>(
      cmd_topic_, rclcpp::QoS(10),
      std::bind(&MissionPlannerNode::onCommand, this, std::placeholders::_1));

    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      std::bind(&MissionPlannerNode::onPlannedPath, this, std::placeholders::_1));

    last_replan_time_ = this->now();

    RCLCPP_INFO(get_logger(), "MissionPlannerNode initialized.");
  }

private:
  // Match with state_machine.hpp
  static constexpr uint8_t CMD_NONE        = 0;
  static constexpr uint8_t CMD_TAKEOFF     = 1;
  static constexpr uint8_t CMD_START       = 2;
  static constexpr uint8_t CMD_HOLD        = 3;
  static constexpr uint8_t CMD_RETURN_HOME = 4;
  static constexpr uint8_t CMD_LAND        = 5;
  static constexpr uint8_t CMD_ABORT       = 6;

  // Params
  std::string receiver_name_;
  std::string cmd_topic_;
  std::string path_topic_;
  double min_replan_period_sec_{0.5};
  double min_goal_change_dist_m_{0.5};

  // ROS
  rclcpp::Subscription<state_machine::msg::Command>::SharedPtr sub_cmd_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;

  // Planner core
  std::unique_ptr<BasicPlanner> planner_;

  // State
  std::mutex mtx_;
  bool exploring_enabled_{false};
  rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d last_goal_{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};

private:
  static inline Eigen::VectorXd toVec3(const Eigen::Vector3d& v)
  {
    Eigen::VectorXd out(3);
    out << v.x(), v.y(), v.z();
    return out;
  }

  static inline double dist3(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
  {
    return (a - b).norm();
  }

  // Toggle exploring mode.
  void setExploringEnabled(bool enabled, const char* reason)
  {
    if (exploring_enabled_ == enabled) return;
    exploring_enabled_ = enabled;

    if (enabled) {
      RCLCPP_INFO(get_logger(), "EXPLORING enabled (%s).", reason);
    } else {
      RCLCPP_INFO(get_logger(), "EXPLORING disabled (%s).", reason);
    }
  }

  void planAndPublishToGoal(const Eigen::Vector3d& goal, const std::string& reason)
  {
    if (!planner_) return;

    const Eigen::Vector3d goal_vel = Eigen::Vector3d::Zero();  // stop at goal

    mav_trajectory_generation::Trajectory traj;
    const bool ok = planner_->planTrajectory(toVec3(goal), toVec3(goal_vel), &traj);
    if (!ok) {
      RCLCPP_WARN(
        get_logger(),
        "Planning failed (%s). Likely no odom on 'current_state_est' yet.",
        reason.c_str());
      return;
    }

    planner_->publishTrajectory(traj);
    RCLCPP_INFO(
      get_logger(),
      "Trajectory published (%s) to goal [%.2f, %.2f, %.2f].",
      reason.c_str(), goal.x(), goal.y(), goal.z());
  }

  // Command callback
  void onCommand(const state_machine::msg::Command::SharedPtr msg)
  {
    if (!msg) return;

    // Only commands addressed to this node
    if (msg->target != receiver_name_) return;

    std::lock_guard<std::mutex> lk(mtx_);

    const uint8_t c = msg->command;

    if (c == CMD_START) {
      if (msg->has_target) {
        // One-shot planning to a specific target point (not exploring)
        setExploringEnabled(false, "START with target");

        const Eigen::Vector3d goal(msg->target_pos.x, msg->target_pos.y, msg->target_pos.z);
        planAndPublishToGoal(goal, "CMD_START target");
      } else {
        // Enter exploring mode; planned_path updates will drive replanning
        setExploringEnabled(true, "START without target");
      }
      return;
    }

    // Safe default: any other command to planner disables exploring
    if (c == CMD_HOLD || c == CMD_ABORT || c == CMD_LAND ||
        c == CMD_RETURN_HOME || c == CMD_TAKEOFF || c == CMD_NONE)
    {
      setExploringEnabled(false, "non-START command");
      return;
    }

    // Unknown numeric command: disable exploring
    setExploringEnabled(false, "unknown command");
  }

  // Path callback (EXPLORING)
  void onPlannedPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg) return;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!exploring_enabled_) return;
    }

    if (msg->poses.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "planned_path is empty.");
      return;
    }

    // Use last pose as the current goal
    const auto& p = msg->poses.back().pose.position;
    const Eigen::Vector3d goal(p.x, p.y, p.z);

    // Throttle replans + only replan if goal changes enough
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const rclcpp::Time now = this->now();
      const double dt = (now - last_replan_time_).seconds();
      if (dt < min_replan_period_sec_) return;

      if (last_goal_.allFinite() && dist3(goal, last_goal_) < min_goal_change_dist_m_) {
        return;
      }

      last_replan_time_ = now;
      last_goal_ = goal;
    }

    planAndPublishToGoal(goal, "planned_path update");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MissionPlannerNode>();
  node->init(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}