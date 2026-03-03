#ifndef EXPLORER_NODE_HPP
#define EXPLORER_NODE_HPP

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <state_machine/msg/answer.hpp>
#include <state_machine/msg/command.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

class FrontierExplorer : public rclcpp::Node {
public: 
    FrontierExplorer();
    ~FrontierExplorer() = default;

private:
    // Callbacks
    void currentStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mapUpdateCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void explorationTimerCallback();
    void commandCallback(const state_machine::msg::Command::SharedPtr msg);
    void healthReportCallback();

    // Frontier detection functions
    void detectFrontiers();
    pcl::PointCloud<pcl::PointXYZ>::Ptr generateFrontierCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr identifyLargestCluster(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointXYZ calculateGoal(pcl::PointCloud<pcl::PointXYZ> &cloud);
    bool isFrontierPoint(const octomap::point3d &coord);
    void publishGoal(const pcl::PointXYZ &goal);
    void checkMovementAndUpdateSearchDistance();

    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_state_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_octomap_;
    rclcpp::Subscription<state_machine::msg::Command>::SharedPtr sub_command_;
    
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_goal_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_marker_;
    rclcpp::Publisher<state_machine::msg::Answer>::SharedPtr pub_node_health_;
    
    rclcpp::TimerBase::SharedPtr timer_explorer_;
    rclcpp::TimerBase::SharedPtr timer_health_report_;

    // State variables
    std::shared_ptr<octomap::OcTree> octree_;
    octomap::point3d curr_pos_;
    octomap::point3d last_pos_;
    rclcpp::Time last_movement_time_;
    
    std::vector<geometry_msgs::msg::Point> frontier_points_;
    std::vector<pcl::PointIndicesPtr> cluster_indices_;

    // Parameters
    double octomap_resolution_{0.4};
    double movement_threshold_{3.0};
    double stall_time_threshold_{10.0};
    int max_distance_{30};
    int max_search_distance_{400};
    double exploration_rate_{0.3};  // Hz
    double health_report_period_{1.0};  // seconds
    bool enforce_x_limit_{false};
    double x_limit_max_{-340.0};
    
    // Control flags
    bool is_exploring_{false};
    bool has_received_pose_{false};
};

#endif // EXPLORER_NODE_HPP
