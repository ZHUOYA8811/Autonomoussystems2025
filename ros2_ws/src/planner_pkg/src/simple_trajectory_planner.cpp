#include "planner_pkg/simple_trajectory_planner.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

SimpleTrajectoryPlanner::SimpleTrajectoryPlanner() 
    : Node("simple_trajectory_planner") 
{
    // Declare parameters
    this->declare_parameter<double>("max_velocity", 3.0);
    this->declare_parameter<double>("max_acceleration", 2.0);
    this->declare_parameter<double>("health_report_period", 1.0);

    // Get parameters
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    max_acceleration_ = this->get_parameter("max_acceleration").as_double();
    health_report_period_ = this->get_parameter("health_report_period").as_double();

    // Initialize publishers
    pub_trajectory_ = this->create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(
        "path_segments_4D", 10);
    pub_node_health_ = this->create_publisher<state_machine::msg::Answer>(
        "statemachine/node_health", 10);

    // Initialize subscribers
    sub_frontier_point_ = this->create_subscription<geometry_msgs::msg::Point>(
        "frontier_point", 10,
        std::bind(&SimpleTrajectoryPlanner::frontierPointCallback, this, _1));

    sub_current_state_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "current_state_est", 10,
        std::bind(&SimpleTrajectoryPlanner::currentStateCallback, this, _1));

    sub_command_ = this->create_subscription<state_machine::msg::Command>(
        "statemachine/cmd", 10,
        std::bind(&SimpleTrajectoryPlanner::commandCallback, this, _1));

    // Health report timer
    auto health_period = std::chrono::duration<double>(health_report_period_);
    timer_health_report_ = this->create_wall_timer(
        health_period,
        std::bind(&SimpleTrajectoryPlanner::healthReportCallback, this));

    // Initialize state
    current_position_ = Eigen::Vector3d::Zero();
    current_velocity_ = Eigen::Vector3d::Zero();

    RCLCPP_INFO(this->get_logger(), "Simple Trajectory Planner initialized");
}

void SimpleTrajectoryPlanner::commandCallback(
    const state_machine::msg::Command::SharedPtr msg) 
{
    if (msg->target != "planner" && msg->target != "sampler") {
        return;
    }

    if (msg->command == 2) {  // START
        if (!is_active_) {
            is_active_ = true;
            RCLCPP_INFO(this->get_logger(), "Trajectory planning activated");
        }
    } else if (msg->command == 3 || msg->command == 6) {  // HOLD or ABORT
        if (is_active_) {
            is_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Trajectory planning deactivated");
        }
    }
}

void SimpleTrajectoryPlanner::healthReportCallback() {
    state_machine::msg::Answer health_msg;
    health_msg.node_name = "planner";
    health_msg.state = is_active_ ? 1 : 0;
    health_msg.info = is_active_ ? "Planning trajectories" : "Idle";
    health_msg.timestamp = this->now();
    pub_node_health_->publish(health_msg);
}

void SimpleTrajectoryPlanner::currentStateCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) 
{
    current_position_ << msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.position.z;

    current_velocity_ << msg->twist.twist.linear.x,
                         msg->twist.twist.linear.y,
                         msg->twist.twist.linear.z;

    has_current_state_ = true;
}

void SimpleTrajectoryPlanner::frontierPointCallback(
    const geometry_msgs::msg::Point::SharedPtr msg) 
{
    if (!is_active_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Received frontier point but planner is not active");
        return;
    }

    if (!has_current_state_) {
        RCLCPP_WARN(this->get_logger(), 
            "Cannot plan trajectory: no current state available");
        return;
    }

    Eigen::Vector3d goal_position(msg->x, msg->y, msg->z);
    
    RCLCPP_INFO(this->get_logger(), 
        "Planning trajectory to frontier: (%.2f, %.2f, %.2f)", 
        msg->x, msg->y, msg->z);

    planTrajectory(goal_position);
}

void SimpleTrajectoryPlanner::planTrajectory(const Eigen::Vector3d& goal_position) {
    mav_trajectory_generation::Trajectory trajectory;

    bool success = generatePolynomialTrajectory(
        current_position_, 
        current_velocity_,
        goal_position,
        trajectory);

    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate trajectory");
        return;
    }

    // Convert to ROS message
    mav_planning_msgs::msg::PolynomialTrajectory4D trajectory_msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
        trajectory, &trajectory_msg);

    trajectory_msg.header.stamp = this->now();
    trajectory_msg.header.frame_id = "world";

    pub_trajectory_->publish(trajectory_msg);

    RCLCPP_INFO(this->get_logger(), 
        "Published trajectory with %zu segments, duration: %.2f s",
        trajectory_msg.segments.size(), trajectory.getMaxTime());
}

bool SimpleTrajectoryPlanner::generatePolynomialTrajectory(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& start_vel,
    const Eigen::Vector3d& goal_pos,
    mav_trajectory_generation::Trajectory& trajectory)
{
    const int derivative_to_optimize = 
        mav_trajectory_generation::derivative_order::ACCELERATION;
    const int polynomial_degree = 10;

    mav_trajectory_generation::Vertex::Vector vertices;

    // Start vertex
    mav_trajectory_generation::Vertex start(4);  // 4D: x, y, z, yaw
    start.makeStartOrEnd(start_pos, derivative_to_optimize);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, start_vel);
    vertices.push_back(start);

    // Goal vertex
    mav_trajectory_generation::Vertex goal(4);
    goal.makeStartOrEnd(goal_pos, derivative_to_optimize);
    goal.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, 
                       Eigen::Vector3d::Zero());
    vertices.push_back(goal);

    // Compute segment times
    std::vector<double> segment_times;
    double distance = (goal_pos - start_pos).norm();
    double estimated_time = std::max(distance / max_velocity_, 2.0);
    segment_times.push_back(estimated_time);

    // Setup optimization
    mav_trajectory_generation::PolynomialOptimization<polynomial_degree> opt(4);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // Solve
    opt.solveLinear();

    // Get trajectory
    mav_trajectory_generation::Segment::Vector segments;
    opt.getSegments(&segments);
    
    if (segments.empty()) {
        return false;
    }

    trajectory = mav_trajectory_generation::Trajectory();
    trajectory.setSegments(segments);

    return true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTrajectoryPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
