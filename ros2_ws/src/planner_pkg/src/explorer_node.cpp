#include "planner_pkg/explorer_node.hpp"
#include "planner_pkg/Optics.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

FrontierExplorer::FrontierExplorer() : Node("frontier_explorer_node") {
    // Declare parameters
    this->declare_parameter<double>("octomap_resolution", 0.4);
    this->declare_parameter<double>("movement_threshold", 3.0);
    this->declare_parameter<double>("stall_time_threshold", 10.0);
    this->declare_parameter<int>("max_distance", 30);
    this->declare_parameter<int>("max_search_distance", 400);
    this->declare_parameter<double>("exploration_rate", 0.3);
    this->declare_parameter<double>("health_report_period", 1.0);
    this->declare_parameter<bool>("enforce_x_limit", false);
    this->declare_parameter<double>("x_limit_max", -340.0);

    // Get parameters
    octomap_resolution_ = this->get_parameter("octomap_resolution").as_double();
    movement_threshold_ = this->get_parameter("movement_threshold").as_double();
    stall_time_threshold_ = this->get_parameter("stall_time_threshold").as_double();
    max_distance_ = this->get_parameter("max_distance").as_int();
    max_search_distance_ = this->get_parameter("max_search_distance").as_int();
    exploration_rate_ = this->get_parameter("exploration_rate").as_double();
    health_report_period_ = this->get_parameter("health_report_period").as_double();
    enforce_x_limit_ = this->get_parameter("enforce_x_limit").as_bool();
    x_limit_max_ = this->get_parameter("x_limit_max").as_double();

    // Initialize publishers
    pub_goal_ = this->create_publisher<geometry_msgs::msg::Point>(
        "frontier_point", 10);
    pub_goal_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "frontier_point_marker", 10);
    pub_node_health_ = this->create_publisher<state_machine::msg::Answer>(
        "statemachine/node_health", 10);

    // Initialize subscribers
    sub_state_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "current_state_est", 10, 
        std::bind(&FrontierExplorer::currentStateCallback, this, _1));
    
    sub_octomap_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "octomap_full", 10,
        std::bind(&FrontierExplorer::mapUpdateCallback, this, _1));
    
    sub_command_ = this->create_subscription<state_machine::msg::Command>(
        "statemachine/cmd", 10,
        std::bind(&FrontierExplorer::commandCallback, this, _1));

    // Initialize timers
    auto exploration_period = std::chrono::duration<double>(1.0 / exploration_rate_);
    timer_explorer_ = this->create_wall_timer(
        exploration_period,
        std::bind(&FrontierExplorer::explorationTimerCallback, this));

    auto health_period = std::chrono::duration<double>(health_report_period_);
    timer_health_report_ = this->create_wall_timer(
        health_period,
        std::bind(&FrontierExplorer::healthReportCallback, this));

    // Initialize state
    last_pos_ = octomap::point3d(0, 0, 0);
    curr_pos_ = octomap::point3d(0, 0, 0);
    last_movement_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "=== Frontier Explorer Node Started ===");
    RCLCPP_INFO(this->get_logger(), "Waiting for START command from state machine...");
}

void FrontierExplorer::commandCallback(const state_machine::msg::Command::SharedPtr msg) {
    // Check if this command is for the planner
    if (msg->target != "planner") {
        return;
    }

    // Command types: 0=NONE, 1=TAKEOFF, 2=START, 3=HOLD, 4=RETURN_HOME, 5=LAND, 6=ABORT
    if (msg->command == 2) {  // START
        is_exploring_ = true;
        RCLCPP_INFO(this->get_logger(), "Received START command - Beginning exploration");
    } else if (msg->command == 3 || msg->command == 6) {  // HOLD or ABORT
        is_exploring_ = false;
        RCLCPP_INFO(this->get_logger(), "Received HOLD/ABORT command - Stopping exploration");
    }
}

void FrontierExplorer::healthReportCallback() {
    state_machine::msg::Answer health_msg;
    health_msg.node_name = "planner";
    health_msg.state = is_exploring_ ? 1 : 0;  // 1=RUNNING, 0=UNKNOWN
    health_msg.info = is_exploring_ ? "Exploring frontiers" : "Idle";
    health_msg.timestamp = this->now();
    pub_node_health_->publish(health_msg);
}

void FrontierExplorer::checkMovementAndUpdateSearchDistance() {
    if (!has_received_pose_) {
        return;
    }

    // Calculate distance between current and last position
    double distance = std::sqrt(
        std::pow(curr_pos_.x() - last_pos_.x(), 2) + 
        std::pow(curr_pos_.y() - last_pos_.y(), 2) + 
        std::pow(curr_pos_.z() - last_pos_.z(), 2));

    rclcpp::Time current_time = this->now();

    if (distance > movement_threshold_) {
        // Robot moved more than threshold
        last_movement_time_ = current_time;
        last_pos_ = curr_pos_;

        // Reset search distance if it was at max
        if (max_distance_ == max_search_distance_) {
            max_distance_ = 30;
            RCLCPP_INFO(this->get_logger(), 
                "Robot started moving, reset search distance to %d meters", max_distance_);
        }
    } else if ((current_time - last_movement_time_).seconds() > stall_time_threshold_ && 
               curr_pos_.x() < -600) {
        // Robot stalled in deep region
        max_distance_ = max_search_distance_;
        RCLCPP_INFO(this->get_logger(), 
            "Robot stalled for %.1f seconds in x < -600 region, set search distance to max %d meters", 
            stall_time_threshold_, max_distance_);
        last_movement_time_ = current_time;
    }
}

void FrontierExplorer::explorationTimerCallback() {
    if (!octree_ || !is_exploring_) {
        return;
    }

    if (!has_received_pose_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Waiting for current position...");
        return;
    }

    // Check movement and update search distance
    checkMovementAndUpdateSearchDistance();

    // Detect frontiers
    detectFrontiers();
    
    if (frontier_points_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No frontier points detected");
        return;
    }

    // Generate point cloud from frontiers
    auto frontier_cloud = generateFrontierCloud();

    // Cluster frontiers using OPTICS
    cluster_indices_.clear();
    Optics::optics<pcl::PointXYZ>(frontier_cloud, 5, 10.0, cluster_indices_);

    if (cluster_indices_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No valid clusters found");
        return;
    }

    // Identify largest cluster and calculate goal
    auto largest_cluster_cloud = identifyLargestCluster(frontier_cloud);
    auto goal = calculateGoal(*largest_cluster_cloud);

    RCLCPP_INFO(this->get_logger(), 
        "Frontier goal: (%.2f, %.2f, %.2f) from %zu frontier points in %zu clusters",
        goal.x, goal.y, goal.z, frontier_points_.size(), cluster_indices_.size());

    publishGoal(goal);
}

void FrontierExplorer::detectFrontiers() {
    frontier_points_.clear();

    octomap::point3d minPt(
        curr_pos_.x() - max_distance_, 
        curr_pos_.y() - max_distance_, 
        curr_pos_.z() - max_distance_);
    
    octomap::point3d maxPt(
        enforce_x_limit_
            ? std::min(static_cast<double>(curr_pos_.x() + max_distance_), x_limit_max_)
            : static_cast<double>(curr_pos_.x() + max_distance_),
        curr_pos_.y() + max_distance_, 
        curr_pos_.z() + max_distance_);

    for (auto it = octree_->begin_leafs_bbx(minPt, maxPt); 
         it != octree_->end_leafs_bbx(); ++it) {
        
        octomap::point3d coord = it.getCoordinate();
        
        // Only consider free space
        if (!octree_->isNodeOccupied(*it)) {
            if (isFrontierPoint(coord)) {
                geometry_msgs::msg::Point frontier_point;
                frontier_point.x = coord.x();
                frontier_point.y = coord.y();
                frontier_point.z = coord.z();
                frontier_points_.push_back(frontier_point);
            }
        }
    }
}

bool FrontierExplorer::isFrontierPoint(const octomap::point3d &coord) {
    // Check 26 neighbors
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dz = -1; dz <= 1; dz++) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                
                octomap::point3d neighbor(
                    coord.x() + dx * octomap_resolution_,
                    coord.y() + dy * octomap_resolution_,
                    coord.z() + dz * octomap_resolution_);
                
                octomap::OcTreeNode* node = octree_->search(neighbor);
                
                // If neighbor is unknown, this is a frontier
                if (node == nullptr) {
                    return true;
                }
            }
        }
    }
    return false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FrontierExplorer::generateFrontierCloud() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = "world";
    cloud->is_dense = false;

    for (const auto &point : frontier_points_) {
        cloud->points.emplace_back(point.x, point.y, point.z);
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FrontierExplorer::identifyLargestCluster(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    size_t max_size = 0;
    pcl::PointIndicesPtr largest_cluster_indices;
    
    for (const auto &cluster_idx : cluster_indices_) {
        if (cluster_idx->indices.size() > max_size) {
            largest_cluster_indices = cluster_idx;
            max_size = cluster_idx->indices.size();
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (largest_cluster_indices) {
        for (int index : largest_cluster_indices->indices) {
            largest_cluster->points.push_back(cloud->points[index]);
        }
    }
    
    return largest_cluster;
}

pcl::PointXYZ FrontierExplorer::calculateGoal(pcl::PointCloud<pcl::PointXYZ> &cloud) {
    pcl::PointXYZ goal(0, 0, 0);
    
    if (cloud.points.empty()) {
        return goal;
    }

    // Calculate centroid of cluster
    for (const auto &point : cloud.points) {
        goal.x += point.x;
        goal.y += point.y;
        goal.z += point.z;
    }
    
    goal.x /= cloud.points.size();
    goal.y /= cloud.points.size();
    goal.z /= cloud.points.size();

    return goal;
}

void FrontierExplorer::currentStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    curr_pos_ = octomap::point3d(
        msg->pose.pose.position.x, 
        msg->pose.pose.position.y, 
        msg->pose.pose.position.z);

    // Initialize last position on first callback
    if (!has_received_pose_) {
        last_pos_ = curr_pos_;
        last_movement_time_ = this->now();
        has_received_pose_ = true;
    }
}

void FrontierExplorer::mapUpdateCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    // Convert message to octomap
    octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
    
    if (abstract_tree) {
        octree_ = std::shared_ptr<octomap::OcTree>(
            dynamic_cast<octomap::OcTree*>(abstract_tree));
        
        if (!octree_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert abstract tree to OcTree");
        }
    }
}

void FrontierExplorer::publishGoal(const pcl::PointXYZ &goal) {
    geometry_msgs::msg::Point goal_msg;
    goal_msg.x = goal.x;
    goal_msg.y = goal.y;
    goal_msg.z = goal.z;

    // Create visualization marker
    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.frame_id = "world";
    goal_marker.header.stamp = this->now();
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.id = 0;
    goal_marker.pose.position.x = goal.x;
    goal_marker.pose.position.y = goal.y;
    goal_marker.pose.position.z = goal.z;
    goal_marker.pose.orientation.x = 0.0;
    goal_marker.pose.orientation.y = 0.0;
    goal_marker.pose.orientation.z = 0.0;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 2.0;
    goal_marker.scale.y = 2.0;
    goal_marker.scale.z = 2.0;
    goal_marker.color.a = 1.0;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 1.0;
    goal_marker.lifetime = rclcpp::Duration::from_seconds(5.0);

    // Optional x-limit gate (disabled by default)
    if (!enforce_x_limit_ || goal_msg.x <= x_limit_max_) {
        pub_goal_->publish(goal_msg);
        pub_goal_marker_->publish(goal_marker);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
