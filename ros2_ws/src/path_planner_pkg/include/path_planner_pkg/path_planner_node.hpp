#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>

// 自定义消息（来自state_machine包）
#include "state_machine/msg/command.hpp"
#include "state_machine/msg/answer.hpp"

#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <random>
#include <functional>
#include <optional>

namespace path_planner {

// ============================================================
// 3D 占用栅格地图
// ============================================================
class OccupancyMap3D {
public:
    OccupancyMap3D(double resolution,
                   double origin_x, double origin_y, double origin_z,
                   int size_x, int size_y, int size_z);

    // 将世界坐标转换为栅格索引
    bool worldToGrid(double wx, double wy, double wz,
                     int& gx, int& gy, int& gz) const;

    // 将栅格索引转换为世界坐标
    void gridToWorld(int gx, int gy, int gz,
                     double& wx, double& wy, double& wz) const;

    // 设置/获取占用值（0=自由, 100=占用, -1=未知）
    void setCell(int gx, int gy, int gz, int8_t value);
    int8_t getCell(int gx, int gy, int gz) const;

    // 更新点云（将点云中的点标记为占用）
    void updateFromPointCloud(const std::vector<Eigen::Vector3d>& points,
                              const Eigen::Vector3d& sensor_origin,
                              double max_range = 10.0);

    // 检查路径是否无碰撞
    bool isPathFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                    double robot_radius = 0.5) const;

    // 检查某点是否在自由空间
    bool isFree(double wx, double wy, double wz) const;
    bool isFreeGrid(int gx, int gy, int gz) const;

    // 获取frontier点（已知自由空间与未知空间的边界）
    std::vector<Eigen::Vector3d> getFrontierPoints() const;

    // 获取无人机附近的frontier点（限制搜索范围，更高效）
    std::vector<Eigen::Vector3d> getFrontierPointsNear(const Eigen::Vector3d& robot_pos, double search_radius = 30.0) const;

    // 地图参数
    double resolution() const { return resolution_; }
    int sizeX() const { return size_x_; }
    int sizeY() const { return size_y_; }
    int sizeZ() const { return size_z_; }
    double originX() const { return origin_x_; }
    double originY() const { return origin_y_; }
    double originZ() const { return origin_z_; }

    // 转换为ROS OccupancyGrid（2D投影，用于可视化）
    nav_msgs::msg::OccupancyGrid toOccupancyGrid2D(double z_min, double z_max) const;

private:
    int index(int gx, int gy, int gz) const {
        return gx + gy * size_x_ + gz * size_x_ * size_y_;
    }
    bool inBounds(int gx, int gy, int gz) const {
        return gx >= 0 && gx < size_x_ &&
               gy >= 0 && gy < size_y_ &&
               gz >= 0 && gz < size_z_;
    }

    double resolution_;
    double origin_x_, origin_y_, origin_z_;
    int size_x_, size_y_, size_z_;
    std::vector<int8_t> data_;  // 栅格数据
};

// ============================================================
// A* 路径规划器
// ============================================================
class AStarPlanner {
public:
    struct Node3D {
        int x, y, z;
        double g_cost;   // 从起点到当前节点的代价
        double h_cost;   // 启发式估计代价
        double f_cost() const { return g_cost + h_cost; }
        int parent_x, parent_y, parent_z;
        bool operator>(const Node3D& other) const {
            return f_cost() > other.f_cost();
        }
    };

    explicit AStarPlanner(const OccupancyMap3D& map, double robot_radius = 0.5);

    // 规划从start到goal的路径，返回路径点列表
    std::vector<Eigen::Vector3d> plan(const Eigen::Vector3d& start,
                                       const Eigen::Vector3d& goal);

    // 设置机器人半径（用于碰撞检测）
    void setRobotRadius(double r) { robot_radius_ = r; }

private:
    double heuristic(int x1, int y1, int z1, int x2, int y2, int z2) const;
    std::vector<Eigen::Vector3d> reconstructPath(
        const std::unordered_map<int, Node3D>& came_from,
        const Node3D& current) const;
    int nodeIndex(int x, int y, int z) const;

    const OccupancyMap3D& map_;
    double robot_radius_;
    // 26-连通邻居方向
    static const std::vector<std::tuple<int,int,int>> NEIGHBORS_26;
};

// ============================================================
// RRT 探索规划器（用于未知区域探索）
// ============================================================
class RRTExplorer {
public:
    struct RRTNode {
        Eigen::Vector3d pos;
        int parent_idx;
        RRTNode(const Eigen::Vector3d& p, int parent)
            : pos(p), parent_idx(parent) {}
    };

    explicit RRTExplorer(const OccupancyMap3D& map,
                          double step_size = 1.5,
                          double goal_bias = 0.1,
                          int max_iter = 5000,
                          double robot_radius = 0.5);

    // 向frontier点规划路径
    std::vector<Eigen::Vector3d> planToFrontier(
        const Eigen::Vector3d& start,
        const std::vector<Eigen::Vector3d>& frontiers);

    // 向指定目标规划路径
    std::vector<Eigen::Vector3d> planToGoal(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& goal);

private:
    Eigen::Vector3d randomSample(const Eigen::Vector3d& goal);
    int nearestNode(const Eigen::Vector3d& point) const;
    Eigen::Vector3d steer(const Eigen::Vector3d& from,
                           const Eigen::Vector3d& to) const;
    std::vector<Eigen::Vector3d> extractPath(int goal_idx) const;

    const OccupancyMap3D& map_;
    double step_size_;
    double goal_bias_;
    int max_iter_;
    double robot_radius_;
    std::vector<RRTNode> tree_;
    std::mt19937 rng_;    std::uniform_real_distribution<double> dist_01_;

    // 空间哈希辅助
    int64_t bucketKey(const Eigen::Vector3d& p) const;
    void hashInsert(int idx);
};

// ============================================================
// 主路径规划节点
// ============================================================
class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode();
    ~PathPlannerNode() = default;

private:
    // ---- 枚举定义 ----
    enum class PlannerState {
        IDLE,       // 等待命令
        EXPLORING,  // 自主探索模式
        NAVIGATING, // 导航到指定目标
        HOLDING,    // 保持当前位置
        ERROR
    };

    enum class Commands : uint8_t {
        NONE = 0,
        TAKEOFF = 1,
        START = 2,
        HOLD = 3,
        RETURN_HOME = 4,
        LAND = 5,
        ABORT = 6
    };

    // ---- 回调函数 ----
    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void onCommand(const state_machine::msg::Command::SharedPtr msg);
    void onTimer();

    // ---- 规划逻辑 ----
    void runExploration();
    void runNavigation(const Eigen::Vector3d& goal);
    void publishPath(const std::vector<Eigen::Vector3d>& path);
    void publishTrajectory(const std::vector<Eigen::Vector3d>& path);
    void publishNodeHealth();
    void publishMapVisualization();
    void publishPathVisualization(const std::vector<Eigen::Vector3d>& path);

    // ---- 辅助函数 ----
    std::vector<Eigen::Vector3d> pointCloud2ToEigen(
        const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    bool hasReachedGoal(const Eigen::Vector3d& goal, double tol = 1.5) const;
    Eigen::Vector3d selectBestFrontier(
        const std::vector<Eigen::Vector3d>& frontiers) const;

    // ---- ROS2 接口 ----
    // 订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
    rclcpp::Subscription<state_machine::msg::Command>::SharedPtr sub_command_;

    // 发布者
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr pub_trajectory_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<state_machine::msg::Answer>::SharedPtr pub_health_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- 状态变量 ----
    PlannerState state_ = PlannerState::IDLE;
    Eigen::Vector3d current_pos_{0.0, 0.0, 0.0};
    Eigen::Vector3d current_vel_{0.0, 0.0, 0.0};
    bool has_odom_ = false;
    bool has_map_ = false;

    // 当前规划路径
    std::vector<Eigen::Vector3d> current_path_;
    int path_index_ = 0;
    Eigen::Vector3d current_goal_;
    bool has_goal_ = false;

    // 地图
    std::unique_ptr<OccupancyMap3D> map_;

    // 规划器
    std::unique_ptr<AStarPlanner> astar_;
    std::unique_ptr<RRTExplorer> rrt_;

    // 互斥锁
    mutable std::mutex map_mutex_;
    mutable std::mutex odom_mutex_;

    // ---- 参数 ----
    double map_resolution_;
    double map_origin_x_, map_origin_y_, map_origin_z_;  // 自动计算
    int map_size_x_, map_size_y_, map_size_z_;            // 自动计算
    double robot_radius_;
    double max_speed_;        // 探索模式速度 (m/s)
    double transit_speed_;    // 巡航/导航模式速度 (m/s)，飞往目标时使用
    double waypoint_reach_dist_;
    double replan_dist_;
    double explore_height_;
    bool use_astar_;   // true=A*, false=RRT
    std::string node_name_in_sm_;  // state_machine中的节点名称
    int nav_fail_count_ = 0;       // 连续规划失败计数（切换逃脱模式用）
    int no_frontier_count_ = 0;    // 连续无frontier计数（触发回退探索）

    // 探索策略参数（来自 planner_pkg）
    double movement_threshold_;     // 卡住检测位移阈值 (m)
    double stall_time_threshold_;   // 卡住检测时间窗口 (s)
    int max_distance_;              // 前沿搜索初始半径（栅格数）
    int max_search_distance_;       // 卡住后扩大搜索的最大半径
    double exploration_rate_;       // 探索频率 (Hz)
    bool enforce_x_limit_;          // 是否限制 X 轴深度
    double x_limit_max_;            // X 轴最深许可坐标
    // 卡住检测状态
    rclcpp::Time last_move_time_;   // 上次有效移动的时间
    Eigen::Vector3d last_move_pos_ = Eigen::Vector3d::Zero();  // 上次记录位置
    bool is_stalled_ = false;       // 当前是否卡住
    int stall_backup_attempts_ = 0; // 卡住后的回退尝试次数

    // 前方障碍预警区（warn zone）：减速系数 + 爬升偏置
    // 由 onPointCloud 设置，由 publishTrajectory 消费
    double obstacle_speed_scale_ = 1.0;  // 减速系数 [0.3, 1.0]，1.0=正常速度
    double obstacle_climb_bias_  = 0.0;  // 目标点 Z 偏置 (m)，0=不爬升

    // 根据当前状态返回合适的飞行速度
    double getEffectiveSpeed() const {
        return (state_ == PlannerState::NAVIGATING) ? transit_speed_ : max_speed_;
    }
};

}  // namespace path_planner
