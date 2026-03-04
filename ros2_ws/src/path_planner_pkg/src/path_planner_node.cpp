#include "path_planner_pkg/path_planner_node.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace path_planner {

PathPlannerNode::PathPlannerNode()
    : rclcpp::Node("path_planner_node")
{
    // ---- 声明参数 ----
    this->declare_parameter<double>("map_resolution", 0.5);
    this->declare_parameter<double>("robot_radius", 0.8);
    this->declare_parameter<double>("max_speed", 2.0);
    this->declare_parameter<double>("transit_speed", 5.0);
    this->declare_parameter<double>("waypoint_reach_dist", 2.0);
    this->declare_parameter<double>("replan_dist", 5.0);
    this->declare_parameter<double>("explore_height", 5.0);
    this->declare_parameter<bool>("use_astar", true);
    this->declare_parameter<std::string>("node_name_in_sm", "navigator");
    // 探索策略参数（来自 planner_pkg）
    this->declare_parameter<double>("movement_threshold", 3.0);
    this->declare_parameter<double>("stall_time_threshold", 10.0);
    this->declare_parameter<int>("max_distance", 30);
    this->declare_parameter<int>("max_search_distance", 400);
    this->declare_parameter<double>("exploration_rate", 0.3);
    this->declare_parameter<bool>("enforce_x_limit", false);
    this->declare_parameter<double>("x_limit_max", -700.0);

    // ---- 读取参数 ----
    map_resolution_  = this->get_parameter("map_resolution").as_double();
    robot_radius_    = this->get_parameter("robot_radius").as_double();
    max_speed_       = this->get_parameter("max_speed").as_double();
    transit_speed_   = this->get_parameter("transit_speed").as_double();
    waypoint_reach_dist_ = this->get_parameter("waypoint_reach_dist").as_double();
    replan_dist_     = this->get_parameter("replan_dist").as_double();
    explore_height_  = this->get_parameter("explore_height").as_double();
    use_astar_       = this->get_parameter("use_astar").as_bool();
    node_name_in_sm_ = this->get_parameter("node_name_in_sm").as_string();
    movement_threshold_   = this->get_parameter("movement_threshold").as_double();
    stall_time_threshold_ = this->get_parameter("stall_time_threshold").as_double();
    max_distance_         = this->get_parameter("max_distance").as_int();
    max_search_distance_  = this->get_parameter("max_search_distance").as_int();
    exploration_rate_     = this->get_parameter("exploration_rate").as_double();
    enforce_x_limit_      = this->get_parameter("enforce_x_limit").as_bool();
    x_limit_max_          = this->get_parameter("x_limit_max").as_double();

    // ---- 自动计算地图范围（无需手动配置） ----
    // 基于分辨率自动生成足够大的地图，覆盖洞穴探索场景
    // X: [-800, +200] = 1000m, Y: [-150, +150] = 300m, Z: [-15, +45] = 60m
    map_origin_x_ = -800.0;
    map_origin_y_ = -150.0;
    map_origin_z_ = -15.0;
    map_size_x_ = static_cast<int>(1000.0 / map_resolution_);  // 2000 cells @ 0.5m
    map_size_y_ = static_cast<int>(300.0 / map_resolution_);   // 600 cells @ 0.5m
    map_size_z_ = static_cast<int>(60.0 / map_resolution_);    // 120 cells @ 0.5m

    // ---- 初始化地图 ----
    map_ = std::make_unique<OccupancyMap3D>(
        map_resolution_,
        map_origin_x_, map_origin_y_, map_origin_z_,
        map_size_x_, map_size_y_, map_size_z_);

    // ---- 初始化规划器 ----
    astar_ = std::make_unique<AStarPlanner>(*map_, robot_radius_);
    rrt_   = std::make_unique<RRTExplorer>(*map_, 1.5, 0.1, 5000, robot_radius_);

    // ---- 初始化卡住检测状态 ----
    last_move_time_ = this->now();
    last_move_pos_ = Eigen::Vector3d::Zero();
    is_stalled_ = false;

    // ---- 创建订阅者 ----
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "current_state_est", rclcpp::QoS(10),
        std::bind(&PathPlannerNode::onOdometry, this, std::placeholders::_1));

    sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/Quadrotor/Sensors/DepthCamera/point_cloud", rclcpp::SensorDataQoS(),
        std::bind(&PathPlannerNode::onPointCloud, this, std::placeholders::_1));

    sub_command_ = this->create_subscription<state_machine::msg::Command>(
        "statemachine/cmd", rclcpp::QoS(10),
        std::bind(&PathPlannerNode::onCommand, this, std::placeholders::_1));

    // ---- 创建发布者 ----
    pub_trajectory_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
        "command/trajectory", rclcpp::QoS(10));

    pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
        "path_planner/path", rclcpp::QoS(10));

    pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "path_planner/map", rclcpp::QoS(1));

    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "path_planner/markers", rclcpp::QoS(10));

    pub_health_ = this->create_publisher<state_machine::msg::Answer>(
        "statemachine/node_health", rclcpp::QoS(10));

    // ---- 创建定时器（10Hz主循环）----
    timer_ = this->create_wall_timer(
        100ms, std::bind(&PathPlannerNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(),
        "PathPlannerNode initialized. Map: %dx%dx%d @ %.2fm (auto-computed, "
        "origin=[%.0f,%.0f,%.0f], mem=%dMB). Algo: %s, stall: %.1fm/%.1fs, x_limit: %s",
        map_size_x_, map_size_y_, map_size_z_, map_resolution_,
        map_origin_x_, map_origin_y_, map_origin_z_,
        (int)(map_size_x_ * map_size_y_ * map_size_z_ / 1000000),
        use_astar_ ? "A*" : "RRT",
        movement_threshold_, stall_time_threshold_,
        enforce_x_limit_ ? "on" : "off");
}

// ============================================================
// 回调：里程计
// ============================================================
void PathPlannerNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_pos_.x() = msg->pose.pose.position.x;
    current_pos_.y() = msg->pose.pose.position.y;
    current_pos_.z() = msg->pose.pose.position.z;
    current_vel_.x() = msg->twist.twist.linear.x;
    current_vel_.y() = msg->twist.twist.linear.y;
    current_vel_.z() = msg->twist.twist.linear.z;
    has_odom_ = true;
}

// ============================================================
// 回调：点云（更新地图）
// ============================================================
void PathPlannerNode::onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!has_odom_) return;

    auto points = pointCloud2ToEigen(msg);
    if (points.empty()) return;

    Eigen::Vector3d sensor_origin;
    Eigen::Vector3d vel;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        sensor_origin = current_pos_;
        vel = current_vel_;
    }

    // [安全] 增大感知范围到20m，为高速飞行和急转弯提供更早的避障信息
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_->updateFromPointCloud(points, sensor_origin, 20.0);
        has_map_ = true;
    }

    // [安全] 紧急近距离障碍检测（速度自适应虚拟保险杠）
    // 1) 计算点云中沿飞行方向锥形区域内最近的障碍物距离
    // 2) 停止距离随速度线性增长：max(1.5, speed*0.8) — 5m/s → 4m
    // 3) [新增] 左右侧方检测 (±70°~±110°)，用于转弯时的侧向保护
    double min_dist_all = std::numeric_limits<double>::max();
    double min_dist_cone = std::numeric_limits<double>::max();
    double min_dist_lateral_left = std::numeric_limits<double>::max();
    double min_dist_lateral_right = std::numeric_limits<double>::max();
    Eigen::Vector3d vel_dir = vel.normalized();
    bool has_vel_dir = (vel.norm() > 0.3);

    // 计算垂直于速度方向的左右向量（用于侧向检测）
    Eigen::Vector3d left_dir(0, 0, 0);
    Eigen::Vector3d right_dir(0, 0, 0);
    if (has_vel_dir) {
        // 水平面上的左向量
        left_dir = Eigen::Vector3d(-vel_dir.y(), vel_dir.x(), 0).normalized();
        right_dir = -left_dir;
    }

    for (const auto& pt : points) {
        Eigen::Vector3d diff = pt - sensor_origin;
        double d = diff.norm();
        if (d < 0.3 || d > 15.0) continue;  // 扩大检测范围到15m，更早发现障碍物（急转弯+高速）

        // 全向最近障碍物
        if (d < min_dist_all) min_dist_all = d;

        // 飞行方向锥形区域（±70°）内最近障碍物 — 进一步扩大角度到70°
        if (has_vel_dir && d < 16.0) {  // 扩大到16m：5m/s时warn_dist=12.5m，须留感知余量
            double cos_angle = diff.dot(vel_dir) / d;
            if (cos_angle > 0.34) {  // cos(70°) ≈ 0.34，更宽的检测锥
                if (d < min_dist_cone) min_dist_cone = d;
            }
        }

        // 侧向检测（左右各±60°扇形）— 大幅扩大转弯保护范围
        if (has_vel_dir && d < 13.0) {  // 13m：覆盖侧向warn_dist（5m/s时≈8.75m）加余量
            Eigen::Vector3d diff_norm = diff / d;
            double cos_left = diff_norm.dot(left_dir);
            double cos_right = diff_norm.dot(right_dir);
            // cos(60°) = 0.5，检测左右±60°内的障碍物
            if (cos_left > 0.5) {
                if (d < min_dist_lateral_left) min_dist_lateral_left = d;
            }
            if (cos_right > 0.5) {
                if (d < min_dist_lateral_right) min_dist_lateral_right = d;
            }
        }
    }

    double speed = vel.norm();
    // 动态停止距离：低速 1.5m → 高速 speed*1.0（急转弯惯性大，需更大制动距离）
    double stop_dist = std::max(1.5, speed * 1.0);
    // 侧向停止距离：更保守（急转弯时侧面暴露更多）
    double lateral_stop_dist = std::max(1.5, speed * 0.7);

    // 条件1：前方锥形区域有障碍且距离 < 停止距离
    bool cone_emergency = has_vel_dir && (min_dist_cone < stop_dist) && (speed > 0.3);
    // 条件2：任何方向 < 1.0m（无条件紧急停止）
    bool proximity_emergency = (min_dist_all < 1.0) && (speed > 0.3);
    // 条件3：[新增] 侧向有障碍且距离 < 侧向停止距离（转弯保护）
    bool lateral_emergency = has_vel_dir && (speed > 0.5) &&
        ((min_dist_lateral_left < lateral_stop_dist) || (min_dist_lateral_right < lateral_stop_dist));

    // 记录侧向信息供日志使用
    double min_lateral = std::min(min_dist_lateral_left, min_dist_lateral_right);

    if ((cone_emergency || proximity_emergency || lateral_emergency) &&
        state_ != PlannerState::HOLDING && state_ != PlannerState::IDLE) {

        double trigger_dist = cone_emergency ? min_dist_cone : 
                              (lateral_emergency ? min_lateral : min_dist_all);
        
        const char* emergency_type = cone_emergency ? "FRONT" : 
                                     (lateral_emergency ? "LATERAL" : "PROXIMITY");
        
        RCLCPP_WARN(this->get_logger(),
            "[EMERGENCY-%s] Obstacle at %.1fm (cone=%.1fm, lateral=%.1fm)! Speed=%.1fm/s. Hover!",
            emergency_type, min_dist_all, min_dist_cone, min_lateral, speed);

        // 智能避障：优先向上避障（洞穴中向上通常最安全），其次横移/后退
        Eigen::Vector3d hover_pt = sensor_origin;
        if (trigger_dist > 1.0 && has_vel_dir) {
            // === 策略1（首选）：向上爬升避障 ===
            // 洞穴中凸出的障碍物（石笋/钟乳石）向上绕过最快最安全
            // 爬升量从1.5m增至2.5m，更有效越过凸出障碍
            Eigen::Vector3d up_pt = sensor_origin + Eigen::Vector3d(0, 0, 2.5) - vel_dir * 0.5;
            // 检查上方是否安全（用点云简单估计，扩大安全检查半径到1.5m）
            bool up_clear = true;
            for (const auto& pt : points) {
                Eigen::Vector3d diff_up = pt - up_pt;
                if (diff_up.norm() < 1.5) { up_clear = false; break; }
            }
            
            if (up_clear && sensor_origin.z() < 18.0) {
                hover_pt = up_pt;
                RCLCPP_INFO(this->get_logger(), "[AVOID] Ascending +2.5m to z=%.1f (preferred)", hover_pt.z());
            } else if (lateral_emergency && !cone_emergency) {
                // === 策略2：侧向障碍 → 向远离障碍物的方向横移 + 向上偏移 ===
                Eigen::Vector3d dodge_dir = (min_dist_lateral_left < min_dist_lateral_right) 
                    ? right_dir : left_dir;
                hover_pt = sensor_origin + dodge_dir * 0.8 + Eigen::Vector3d(0, 0, 1.0) - vel_dir * 0.3;
                RCLCPP_INFO(this->get_logger(), "[AVOID] Lateral dodge+up: %s side, z=%.1f", 
                    (min_dist_lateral_left < min_dist_lateral_right) ? "right" : "left", hover_pt.z());
            } else {
                // === 策略3：前方障碍 → 后退 + 向上偏移（增大向上偏移量）===
                hover_pt = sensor_origin - vel_dir * 0.8 + Eigen::Vector3d(0, 0, 1.5);
                RCLCPP_INFO(this->get_logger(), "[AVOID] Backup+up to z=%.1f", hover_pt.z());
            }
        }

        std::vector<Eigen::Vector3d> hover = {hover_pt};
        publishTrajectory(hover);
        // 清空当前路径，强制下一轮重新规划
        current_path_.clear();
        path_index_ = 0;
    }

    // ---- 预警区（warn zone）：障碍物在 [stop_dist, warn_dist] 之间时预减速 + 爬升 ----
    // 策略：比硬制动更早介入，先线性降速，同时微微抬升下一目标点高度，
    //       让无人机在转弯或飞近障碍时有足够时间减速并从上方绕过凸出物。
    // 注意：紧急区内 min_dist_cone < stop_dist → t_warn ≤ 0 → scale = 0.3（最低速）
    //       紧急解除后 scale 随距离增大自然恢复，不会瞬间跳回全速。
    {
        if (has_vel_dir && speed > 0.3) {
            // warn_dist = stop_dist 的 2.5 倍（且至少比 stop_dist 大 5m）
            double warn_dist         = std::max(stop_dist * 2.5, stop_dist + 5.0);
            double lateral_warn_dist = std::max(lateral_stop_dist * 2.5, lateral_stop_dist + 5.0);

            // 前方 warn zone
            double t_front = 1.0;
            if (min_dist_cone < warn_dist) {
                t_front = (min_dist_cone - stop_dist) / (warn_dist - stop_dist);
                t_front = std::max(0.0, std::min(1.0, t_front));
            }

            // 侧向 warn zone（转弯保护）
            double t_lateral = 1.0;
            if (min_lateral < lateral_warn_dist) {
                t_lateral = (min_lateral - lateral_stop_dist) / (lateral_warn_dist - lateral_stop_dist);
                t_lateral = std::max(0.0, std::min(1.0, t_lateral));
            }

            // 取最保守的（最小 t = 最需要减速）
            double t_warn = std::min(t_front, t_lateral);

            double new_scale  = std::max(0.3, t_warn);        // 最低 30% 速度
            double new_climb  = (1.0 - t_warn) * 2.0;         // 越近爬升越多，最多 +2m

            // 单调递减滤波：scale 只往下走（检测到障碍立即减速），
            // 往上恢复时以 0.05/帧（约 0.5 m/s² 等效）限速，避免瞬间满速冲回
            if (new_scale < obstacle_speed_scale_) {
                obstacle_speed_scale_ = new_scale;   // 立即减速
            } else {
                obstacle_speed_scale_ = std::min(new_scale,
                    obstacle_speed_scale_ + 0.05);   // 缓慢恢复
            }
            obstacle_climb_bias_ = new_climb;

            if (new_scale < 0.99) {
                RCLCPP_DEBUG(this->get_logger(),
                    "[WARN_ZONE] cone=%.1fm lat=%.1fm  decel=%.0f%%  climb=+%.1fm",
                    min_dist_cone, min_lateral,
                    obstacle_speed_scale_ * 100.0, obstacle_climb_bias_);
            }
        } else {
            // 无速度方向时缓慢恢复（避免静止时锁住速度）
            obstacle_speed_scale_ = std::min(1.0, obstacle_speed_scale_ + 0.05);
            obstacle_climb_bias_  = 0.0;
        }
    }
}

// ============================================================
// 回调：来自state_machine的命令
// ============================================================
void PathPlannerNode::onCommand(const state_machine::msg::Command::SharedPtr msg)
{
    if (!msg) return;

    // 只处理发给本节点的命令
    if (msg->target != node_name_in_sm_ && msg->target != "planner") {
        return;
    }

    Commands cmd = static_cast<Commands>(msg->command);

    switch (cmd) {
        case Commands::START:
            if (msg->has_target) {
                // 导航到指定目标
                current_goal_ = Eigen::Vector3d(
                    msg->target_pos.x,
                    msg->target_pos.y,
                    msg->target_pos.z);
                has_goal_ = true;
                state_ = PlannerState::NAVIGATING;
                current_path_.clear();
                path_index_ = 0;
                RCLCPP_INFO(this->get_logger(),
                    "Received NAVIGATE command to [%.2f, %.2f, %.2f]",
                    current_goal_.x(), current_goal_.y(), current_goal_.z());
            } else {
                // 自主探索模式 — 如果已经在探索，不要重置路径
                if (state_ != PlannerState::EXPLORING) {
                    state_ = PlannerState::EXPLORING;
                    has_goal_ = false;
                    current_path_.clear();
                    path_index_ = 0;
                    RCLCPP_INFO(this->get_logger(),
                        "Received EXPLORE command, starting autonomous exploration");
                }
            }
            break;

        case Commands::HOLD:
            state_ = PlannerState::HOLDING;
            RCLCPP_INFO(this->get_logger(), "Received HOLD command");
            // 发布当前位置作为目标（悬停）
            if (has_odom_) {
                std::vector<Eigen::Vector3d> hover_path = {current_pos_};
                publishTrajectory(hover_path);
            }
            break;

        case Commands::ABORT:
            state_ = PlannerState::IDLE;
            RCLCPP_INFO(this->get_logger(), "Received ABORT command");
            break;

        case Commands::LAND:
            if (msg->has_target) {
                current_goal_ = Eigen::Vector3d(
                    msg->target_pos.x,
                    msg->target_pos.y,
                    msg->target_pos.z);
                has_goal_ = true;
                state_ = PlannerState::NAVIGATING;
                current_path_.clear();
                path_index_ = 0;
                RCLCPP_INFO(this->get_logger(),
                    "Received LAND command, navigating to [%.2f, %.2f, %.2f]",
                    current_goal_.x(), current_goal_.y(), current_goal_.z());
            }
            break;

        default:
            break;
    }
}

// ============================================================
// 主定时器循环
// ============================================================
void PathPlannerNode::onTimer()
{
    // 发布心跳
    publishNodeHealth();

    if (!has_odom_) return;

    Eigen::Vector3d pos;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        pos = current_pos_;
    }

    switch (state_) {
        case PlannerState::EXPLORING:
            runExploration();
            break;

        case PlannerState::NAVIGATING:
            if (has_goal_) {
                runNavigation(current_goal_);
            }
            break;

        case PlannerState::HOLDING:
            // 保持当前位置，定期重发轨迹
            {
                static int hold_counter = 0;
                if (++hold_counter >= 10) {  // 每秒重发一次
                    hold_counter = 0;
                    std::vector<Eigen::Vector3d> hover_path = {pos};
                    publishTrajectory(hover_path);
                }
            }
            break;

        case PlannerState::IDLE:
        case PlannerState::ERROR:
        default:
            break;
    }

    // 定期发布地图可视化（每5秒）
    static int map_counter = 0;
    if (++map_counter >= 50) {
        map_counter = 0;
        if (has_map_) {
            publishMapVisualization();
        }
    }
}

// ============================================================
// 自主探索逻辑
// ============================================================
void PathPlannerNode::runExploration()
{
    if (!has_odom_) return;

    Eigen::Vector3d pos;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        pos = current_pos_;
    }

    // ---- 卡住检测（来自 planner_pkg 的 explorer_node 策略） ----
    double dist_since_last = (pos - last_move_pos_).norm();
    if (dist_since_last > movement_threshold_) {
        last_move_time_ = this->now();
        last_move_pos_ = pos;
        is_stalled_ = false;
        stall_backup_attempts_ = 0;  // 重置回退尝试计数
    } else {
        double elapsed = (this->now() - last_move_time_).seconds();
        if (elapsed > stall_time_threshold_) {
            if (!is_stalled_) {
                RCLCPP_WARN(this->get_logger(),
                    "Stall detected! Moved only %.1fm in %.1fs (threshold: %.1fm in %.1fs)",
                    dist_since_last, elapsed,
                    movement_threshold_, stall_time_threshold_);
                is_stalled_ = true;
            }
            
            // [新增] 主动回退逃脱机制 - 当卡住超过阈值时立即后退
            if (stall_backup_attempts_ < 5) {
                stall_backup_attempts_++;
                RCLCPP_WARN(this->get_logger(),
                    "[STALL ESCAPE] Active backup attempt %d/5", stall_backup_attempts_);
                
                // 获取当前速度方向，沿相反方向后退
                Eigen::Vector3d vel_dir;
                {
                    std::lock_guard<std::mutex> lock(odom_mutex_);
                    vel_dir = current_vel_.normalized();
                }
                
                // 根据尝试次数增加后退距离和角度变化
                double backup_dist = 1.5 + stall_backup_attempts_ * 0.5;  // 1.5~4.0m
                double lateral_angle = (stall_backup_attempts_ % 2 == 0) ? 0.5 : -0.5;  // 左右交替
                
                Eigen::Vector3d backup_pos = pos;
                if (vel_dir.norm() > 0.1) {
                    backup_pos = pos - vel_dir * backup_dist;
                } else {
                    // 没有明确速度方向，向+X回退（离开洞穴深处）
                    backup_pos = Eigen::Vector3d(pos.x() + backup_dist, pos.y(), pos.z());
                }
                
                // 添加侧向偏移避免卡在同一位置
                backup_pos.y() += lateral_angle * backup_dist * 0.5;
                backup_pos.z() = std::max(explore_height_ - 2.0, std::min(explore_height_ + 2.0, backup_pos.z()));
                
                // 检查回退路径是否安全
                bool backup_safe = true;
                {
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    if (has_map_ && !map_->isPathFree(pos, backup_pos, robot_radius_ * 0.5)) {
                        // 回退路径被阻挡，尝试其他方向
                        for (double angle : {M_PI/4, -M_PI/4, M_PI/2, -M_PI/2, M_PI}) {
                            Eigen::Vector3d alt_backup(
                                pos.x() + std::cos(angle) * backup_dist,
                                pos.y() + std::sin(angle) * backup_dist,
                                pos.z());
                            if (map_->isPathFree(pos, alt_backup, robot_radius_ * 0.3)) {
                                backup_pos = alt_backup;
                                backup_safe = true;
                                break;
                            }
                        }
                    }
                }
                
                if (backup_safe) {
                    std::vector<Eigen::Vector3d> backup_path = {backup_pos};
                    current_path_.clear();
                    path_index_ = 0;
                    publishTrajectory(backup_path);
                    // 重置卡住时间让系统有时间执行回退
                    last_move_time_ = this->now();
                    return;  // 跳过本次规划，等待回退完成
                }
            }
        }
    }

    // ---- X 轴深度限制（来自 planner_pkg 的 enforce_x_limit） ----
    if (enforce_x_limit_ && pos.x() < x_limit_max_) {
        RCLCPP_WARN(this->get_logger(),
            "X-limit reached: x=%.1f < limit=%.1f, retreating",
            pos.x(), x_limit_max_);
        Eigen::Vector3d retreat(pos.x() + 10.0, pos.y(), explore_height_);
        std::vector<Eigen::Vector3d> path = {pos, retreat};
        current_path_ = path;
        path_index_ = 0;
        publishPathVisualization(path);
        publishTrajectory(path);
        return;
    }

    // 检查是否需要重新规划
    bool need_replan = current_path_.empty();

    if (!need_replan && path_index_ < static_cast<int>(current_path_.size())) {
        // 检查是否到达当前航点
        double dist = (pos - current_path_[path_index_]).norm();
        if (dist < waypoint_reach_dist_) {
            path_index_++;
            RCLCPP_DEBUG(this->get_logger(),
                "Reached waypoint %d/%zu", path_index_,
                current_path_.size());
        }

        // 检查路径是否已走完
        if (path_index_ >= static_cast<int>(current_path_.size())) {
            need_replan = true;
        }

        // [安全] 实时碰撞检测：检查当前路径段以及后续4段是否仍然安全
        // 多段前瞻可在转弯前就发现凸出的障碍物，而不是到了弯口才发现
        // [改进] 在转弯处使用更大的碰撞检查半径
        if (!need_replan && has_map_ && path_index_ < static_cast<int>(current_path_.size())) {
            std::lock_guard<std::mutex> lock(map_mutex_);
            int lookahead_end = std::min(path_index_ + 7,
                                         static_cast<int>(current_path_.size()));
            Eigen::Vector3d check_from = pos;
            Eigen::Vector3d prev_dir(0, 0, 0);
            
            for (int li = path_index_; li < lookahead_end; ++li) {
                // 计算转弯角度以决定使用的碰撞检查半径
                Eigen::Vector3d curr_dir = (current_path_[li] - check_from).normalized();
                double effective_radius = robot_radius_;
                
                if (prev_dir.norm() > 0.5 && curr_dir.norm() > 0.5) {
                    double cos_turn = prev_dir.dot(curr_dir);
                    if (cos_turn < 0.7) {  // 转弯角度 > 45°
                        // 在急转弯处增大碰撞检查半径 (最多增加80%)
                        double radius_mult = 1.0 + 0.8 * (0.7 - cos_turn) / 0.7;
                        effective_radius = robot_radius_ * radius_mult;
                    }
                }
                
                if (!map_->isPathFree(check_from, current_path_[li], effective_radius)) {
                    RCLCPP_WARN(this->get_logger(),
                        "[SAFETY] Path segment %d/%zu blocked (explore lookahead, r=%.2f). Replanning...",
                        li, current_path_.size(), effective_radius);
                    need_replan = true;
                    current_path_.clear();
                    break;
                }
                prev_dir = curr_dir;
                check_from = current_path_[li];
            }
        }
    }

    if (!need_replan) {
        // 继续执行当前路径
        if (path_index_ < static_cast<int>(current_path_.size())) {
            // 发布从当前位置到路径终点的轨迹
            std::vector<Eigen::Vector3d> remaining_path(
                current_path_.begin() + path_index_,
                current_path_.end());
            publishTrajectory(remaining_path);
        }
        return;
    }

    // 需要重新规划：寻找frontier点或无地图直接前进
    if (!has_map_) {
        // ====== 无地图模式：小步谨慎前进 ======
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "No map data - cautious forward exploration");

        static int blind_explore_step_ = 0;
        blind_explore_step_++;

        // [安全] 无地图时只前进 4m（之前是15m），避免盲飞撞墙
        double step_len = 4.0;
        double lateral_range = 2.0;
        double vertical_range = 1.0;

        srand(time(NULL) + blind_explore_step_);
        double lateral_offset = ((double)rand() / RAND_MAX * 2.0 - 1.0) * lateral_range;
        double vertical_offset = ((double)rand() / RAND_MAX * 2.0 - 1.0) * vertical_range;

        double next_x = pos.x() - step_len;
        double next_y = pos.y() + lateral_offset;
        double next_z = pos.z() + vertical_offset;

        // 安全限制（使用 explore_height_ 作为基准，防止绝对高度设置导致洞内撞地）
        next_y = std::max(5.0, std::min(15.0, next_y));
        next_z = std::max(explore_height_ - 3.0, std::min(explore_height_ + 5.0, next_z));

        Eigen::Vector3d forward_goal(next_x, next_y, next_z);
        std::vector<Eigen::Vector3d> path = {pos, forward_goal};

        current_path_ = path;
        path_index_ = 0;

        RCLCPP_INFO(this->get_logger(),
            "Blind explore step %d: [%.1f,%.1f,%.1f] -> [%.1f,%.1f,%.1f]",
            blind_explore_step_,
            pos.x(), pos.y(), pos.z(),
            forward_goal.x(), forward_goal.y(), forward_goal.z());

        publishPathVisualization(path);
        publishTrajectory(path);
        return;
    }

    std::vector<Eigen::Vector3d> frontiers;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        // 优先搜索无人机附近30m范围的frontier，避免遍历巨大全地图
        frontiers = map_->getFrontierPointsNear(pos, 30.0);
        // 如果附近没找到，扩大范围到 60m
        if (frontiers.empty()) {
            frontiers = map_->getFrontierPointsNear(pos, 60.0);
        }
    }

    // X 轴深度限制过滤 frontier
    if (enforce_x_limit_ && !frontiers.empty()) {
        size_t before = frontiers.size();
        frontiers.erase(
            std::remove_if(frontiers.begin(), frontiers.end(),
                [this](const Eigen::Vector3d& f) { return f.x() < x_limit_max_; }),
            frontiers.end());
        if (frontiers.size() != before) {
            RCLCPP_INFO(this->get_logger(),
                "X-limit filtered frontiers: %zu -> %zu", before, frontiers.size());
        }
    }

    RCLCPP_INFO(this->get_logger(),
        "Found %zu frontier points, replanning...", frontiers.size());

    if (frontiers.empty()) {
        // 没有frontier：先修正高度，再系统性探索
        no_frontier_count_++;

        RCLCPP_WARN(this->get_logger(),
            "No frontier points found (count=%d, stalled=%s), systematic exploration",
            no_frontier_count_, is_stalled_ ? "YES" : "no");

        // 卡住时加速回退/扩大搜索
        if (is_stalled_ && no_frontier_count_ <= 5) {
            no_frontier_count_ = 6;  // 直接进入回退模式
        }

        // ====== 高度修正：防止越飞越低 ======
        // 如果当前高度偏离 explore_height_ 超过 1m，优先修正高度
        double target_z = explore_height_;
        double z_error = target_z - pos.z();
        bool need_altitude_fix = std::abs(z_error) > 1.0;

        Eigen::Vector3d forward_goal = pos;

        if (need_altitude_fix) {
            // 高度修正模式：先拉回到探索高度
            double z_step = std::copysign(std::min(2.0, std::abs(z_error)), z_error);
            forward_goal = Eigen::Vector3d(pos.x(), pos.y(), pos.z() + z_step);
            forward_goal.z() = std::max(3.0, std::min(20.0, forward_goal.z()));

            RCLCPP_INFO(this->get_logger(),
                "Altitude correction: z=%.1f -> %.1f (target=%.1f)",
                pos.z(), forward_goal.z(), target_z);

            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                if (!map_->isPathFree(pos, forward_goal, robot_radius_)) {
                    // 如果直接升高被阻，略微偏移水平位置后升高
                    bool found_alt = false;
                    for (double dx : {1.0, -1.0, 0.0}) {
                        for (double dy : {0.0, 1.0, -1.0}) {
                            Eigen::Vector3d alt(pos.x() + dx, pos.y() + dy, pos.z() + z_step);
                            alt.z() = std::max(3.0, std::min(20.0, alt.z()));
                            if (map_->isPathFree(pos, alt, 0.3)) {
                                forward_goal = alt;
                                found_alt = true;
                                break;
                            }
                        }
                        if (found_alt) break;
                    }
                    // 实在不行就保持当前高度
                    if (!found_alt) forward_goal = pos;
                }
            }
        } else {
            // ====== 系统性探索模式 ======
            // 连续无 frontier 超过5次：回退（+X方向）寻找新区域
            // 否则：继续谨慎前进（-X）
            double step = 5.0;
            double lateral_offset = ((double)rand() / RAND_MAX * 2.0 - 1.0) * 2.0;
            // 高度偏移：始终趋向 explore_height_，而非纯随机
            double vertical_offset = (target_z - pos.z()) * 0.3
                + ((double)rand() / RAND_MAX * 2.0 - 1.0) * 0.5;

            if (no_frontier_count_ > 5) {
                // 回退模式：向 -X 方向深入洞穴探索，寻找新区域
                double retreat_step = std::min(10.0, no_frontier_count_ * 2.0);
                double retreat_lateral = ((double)rand() / RAND_MAX * 2.0 - 1.0) * 3.0;
                forward_goal = Eigen::Vector3d(
                    pos.x() - retreat_step,    // -X 深入洞穴
                    pos.y() + retreat_lateral,
                    target_z);
                RCLCPP_WARN(this->get_logger(),
                    "Deep exploration: moving -X by %.1f (no_frontier=%d)",
                    retreat_step, no_frontier_count_);

                // 回退成功后重置计数
                if (no_frontier_count_ > 15) no_frontier_count_ = 0;
            } else {
                forward_goal = Eigen::Vector3d(
                    pos.x() - step,
                    pos.y() + lateral_offset,
                    pos.z() + vertical_offset);
            }

            forward_goal.y() = std::max(5.0, std::min(15.0, forward_goal.y()));
            forward_goal.z() = std::max(5.0, std::min(18.0, forward_goal.z()));

            // [安全] 碰撞检查与逃脱策略
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                if (!map_->isPathFree(pos, forward_goal, robot_radius_)) {
                    RCLCPP_WARN(this->get_logger(),
                        "Forward path blocked! Trying alternative directions...");

                    bool found_safe = false;

                    // 第1级：8方向 × 3m，保持目标高度
                    for (int attempt = 0; attempt < 8 && !found_safe; ++attempt) {
                        double angle = attempt * M_PI / 4.0;
                        Eigen::Vector3d alt_goal(
                            pos.x() + std::cos(angle) * 3.0,
                            pos.y() + std::sin(angle) * 3.0,
                            std::max(pos.z(), target_z));  // 不降低高度
                        if (map_->isPathFree(pos, alt_goal, robot_radius_)) {
                            forward_goal = alt_goal;
                            found_safe = true;
                            RCLCPP_INFO(this->get_logger(),
                                "Escape L1: angle=%.0f deg, z=%.1f",
                                angle * 180.0 / M_PI, alt_goal.z());
                        }
                    }

                    // 第2级：8方向 × 1.5m + 向上偏移优先，半径缩至0.3m
                    if (!found_safe) {
                        double dz_arr[] = {1.0, 1.5, 2.0, 0.0, -0.5};  // 优先向上
                        for (double dz : dz_arr) {
                            for (int attempt = 0; attempt < 8 && !found_safe; ++attempt) {
                                double angle = attempt * M_PI / 4.0;
                                Eigen::Vector3d alt_goal(
                                    pos.x() + std::cos(angle) * 1.5,
                                    pos.y() + std::sin(angle) * 1.5,
                                    std::max(5.0, std::min(20.0, pos.z() + dz)));
                                if (map_->isPathFree(pos, alt_goal, 0.3)) {
                                    forward_goal = alt_goal;
                                    found_safe = true;
                                    RCLCPP_INFO(this->get_logger(),
                                        "Escape L2: angle=%.0f dz=%.1f",
                                        angle * 180.0 / M_PI, dz);
                                }
                            }
                        }
                    }

                    // 第3级：沿洞穴深入方向(-X)以递减步长
                    if (!found_safe) {
                        for (double step_try : {3.0, 2.0, 1.0, 0.5}) {
                            for (double dz : {1.0, 0.5, 0.0, -0.5}) {
                                Eigen::Vector3d cave_goal(
                                    pos.x() - step_try,  // -X深入洞穴方向
                                    pos.y(),
                                    std::max(5.0, std::min(20.0, pos.z() + dz)));
                                if (map_->isPathFree(pos, cave_goal, 0.2)) {
                                    forward_goal = cave_goal;
                                    found_safe = true;
                                    RCLCPP_WARN(this->get_logger(),
                                        "Escape L3: retreat +X step=%.2f dz=%.1f",
                                        step_try, dz);
                                    break;
                                }
                            }
                            if (found_safe) break;
                        }
                    }

                    // 第4级：无条件上升 2m（洞穴内最后手段）
                    if (!found_safe) {
                        Eigen::Vector3d up_goal(pos.x(), pos.y(),
                            std::min(20.0, pos.z() + 2.0));
                        forward_goal = up_goal;
                        RCLCPP_WARN(this->get_logger(),
                            "Escape L4: forced ascent to z=%.1f", up_goal.z());
                        found_safe = true;
                    }
                }
            }
        }

        std::vector<Eigen::Vector3d> path = {pos, forward_goal};

        current_path_ = path;
        path_index_ = 0;

        RCLCPP_INFO(this->get_logger(),
            "Safe fly: [%.1f,%.1f,%.1f] -> [%.1f,%.1f,%.1f]",
            pos.x(), pos.y(), pos.z(),
            forward_goal.x(), forward_goal.y(), forward_goal.z());

        publishPathVisualization(path);
        publishTrajectory(path);
        return;
    }

    // 有frontier，重置无frontier计数
    no_frontier_count_ = 0;

    // 选择最佳frontier，并对多个候选尝试规划（避免选到不可达的frontier）
    // 按评分排序，依次尝试直到规划成功
    std::vector<std::pair<double, Eigen::Vector3d>> scored_frontiers;
    for (const auto& f : frontiers) {
        double dist = (f - pos).norm();
        double dz = std::abs(f.z() - pos.z());
        if (dist < 2.0) continue;
        double dz_penalty = (dz > 4.0) ? -50.0 : (dz > 2.0 ? -dz * 2.0 : 0.0);
        double score = -std::abs(dist - 8.0) / 8.0 - dz / 3.0 + dz_penalty;
        scored_frontiers.emplace_back(score, f);
    }
    std::sort(scored_frontiers.begin(), scored_frontiers.end(),
        [](const auto& a, const auto& b){ return a.first > b.first; });

    std::vector<Eigen::Vector3d> path;
    Eigen::Vector3d chosen_frontier = pos;
    bool planning_succeeded = false;

    // 最多尝试前5个候选 frontier
    int max_try = std::min(5, static_cast<int>(scored_frontiers.size()));
    for (int fi = 0; fi < max_try && !planning_succeeded; ++fi) {
        const Eigen::Vector3d& candidate = scored_frontiers[fi].second;
        std::vector<Eigen::Vector3d> try_path;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (use_astar_) {
                try_path = astar_->plan(pos, candidate);
            } else {
                try_path = rrt_->planToGoal(pos, candidate);
            }
        }
        if (!try_path.empty()) {
            path = try_path;
            chosen_frontier = candidate;
            planning_succeeded = true;
            RCLCPP_INFO(this->get_logger(),
                "Frontier #%d planned: [%.1f,%.1f,%.1f] dist=%.1f dz=%.1f",
                fi, candidate.x(), candidate.y(), candidate.z(),
                (candidate - pos).norm(), std::abs(candidate.z() - pos.z()));
        }
    }

    if (planning_succeeded) {
        current_path_ = path;
        path_index_ = 0;
        publishPathVisualization(path);
        publishTrajectory(path);
    } else {
        // 所有候选 frontier 均不可达 → 切换到逃脱模式（前向小步探索）
        RCLCPP_WARN(this->get_logger(),
            "All frontiers unreachable! Switching to escape mode.");
        // 强制触发一次前向小步（沿洞穴 -X 方向，使用最小半径）
        for (double step_try : {3.0, 2.0, 1.0, 0.5}) {
            for (double dz : {0.0, 0.5, -0.5, 1.0, -1.0}) {
                Eigen::Vector3d escape_goal(
                    pos.x() - step_try,
                    pos.y(),
                    std::max(3.0, std::min(20.0, pos.z() + dz)));
                std::lock_guard<std::mutex> lock(map_mutex_);
                if (map_->isPathFree(pos, escape_goal, 0.2)) {
                    current_path_ = {pos, escape_goal};
                    path_index_ = 0;
                    publishTrajectory(current_path_);
                    RCLCPP_WARN(this->get_logger(),
                        "Escape to [%.1f,%.1f,%.1f]",
                        escape_goal.x(), escape_goal.y(), escape_goal.z());
                    return;
                }
            }
        }
        // 最后手段：原地上升
        Eigen::Vector3d up(pos.x(), pos.y(), std::min(20.0, pos.z() + 1.0));
        current_path_ = {pos, up};
        path_index_ = 0;
        publishTrajectory(current_path_);
        RCLCPP_WARN(this->get_logger(), "Last resort: ascending to z=%.1f", up.z());
    }
}

// ============================================================
// 导航到指定目标
// ============================================================
void PathPlannerNode::runNavigation(const Eigen::Vector3d& goal)
{
    if (!has_odom_) return;

    Eigen::Vector3d pos;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        pos = current_pos_;
    }

    // 检查是否已到达目标
    if (hasReachedGoal(goal)) {
        RCLCPP_INFO(this->get_logger(),
            "Reached goal [%.2f, %.2f, %.2f]",
            goal.x(), goal.y(), goal.z());
        state_ = PlannerState::HOLDING;
        return;
    }

    // 检查是否需要重新规划
    bool need_replan = current_path_.empty();

    if (!need_replan && path_index_ < static_cast<int>(current_path_.size())) {
        double dist = (pos - current_path_[path_index_]).norm();
        if (dist < waypoint_reach_dist_) {
            path_index_++;
        }
        if (path_index_ >= static_cast<int>(current_path_.size())) {
            need_replan = true;
        }

        // [安全] 实时碰撞检测：检查当前路径段以及后续4段是否仍然无障碍
        // [改进] 在转弯处使用更大的碰撞检查半径
        if (!need_replan && has_map_ && path_index_ < static_cast<int>(current_path_.size())) {
            std::lock_guard<std::mutex> lock(map_mutex_);
            int lookahead_end = std::min(path_index_ + 7,
                                         static_cast<int>(current_path_.size()));
            Eigen::Vector3d check_from = pos;
            Eigen::Vector3d prev_dir(0, 0, 0);
            
            for (int li = path_index_; li < lookahead_end; ++li) {
                // 计算转弯角度以决定使用的碰撞检查半径
                Eigen::Vector3d curr_dir = (current_path_[li] - check_from).normalized();
                double effective_radius = robot_radius_;
                
                if (prev_dir.norm() > 0.5 && curr_dir.norm() > 0.5) {
                    double cos_turn = prev_dir.dot(curr_dir);
                    if (cos_turn < 0.7) {  // 转弯角度 > 45°
                        // 在急转弯处增大碰撞检查半径 (最多增加80%)
                        double radius_mult = 1.0 + 0.8 * (0.7 - cos_turn) / 0.7;
                        effective_radius = robot_radius_ * radius_mult;
                    }
                }
                
                if (!map_->isPathFree(check_from, current_path_[li], effective_radius)) {
                    RCLCPP_WARN(this->get_logger(),
                        "[SAFETY] Navigation path segment %d blocked (lookahead, r=%.2f). Emergency replan.",
                        li, effective_radius);
                    need_replan = true;
                    current_path_.clear();
                    break;
                }
                prev_dir = curr_dir;
                check_from = current_path_[li];
            }
        }
    }

    if (!need_replan) {
        if (path_index_ < static_cast<int>(current_path_.size())) {
            std::vector<Eigen::Vector3d> remaining(
                current_path_.begin() + path_index_,
                current_path_.end());
            publishTrajectory(remaining);
        }
        return;
    }

    // 规划到目标的路径
    RCLCPP_INFO(this->get_logger(),
        "Planning path to [%.2f, %.2f, %.2f]",
        goal.x(), goal.y(), goal.z());

    std::vector<Eigen::Vector3d> path;

    if (has_map_) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (use_astar_) {
            path = astar_->plan(pos, goal);
        } else {
            path = rrt_->planToGoal(pos, goal);
        }
    }

    if (path.empty()) {
        // [安全] 规划失败时：不再直接飞向目标，而是检查是否安全
        RCLCPP_WARN(this->get_logger(),
            "Path planning failed! Checking if direct path is safe...");

        bool direct_safe = false;
        if (has_map_) {
            std::lock_guard<std::mutex> lock(map_mutex_);
            direct_safe = map_->isPathFree(pos, goal, robot_radius_);
        }

        if (direct_safe) {
            RCLCPP_INFO(this->get_logger(),
                "Direct path to goal is obstacle-free, using it.");
            path = {pos, goal};
            nav_fail_count_ = 0;
        } else {
            // 连续失败计数
            nav_fail_count_++;

            if (nav_fail_count_ < 15) {
                // 短期：悬停等地图更新
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Direct path BLOCKED! Hold (%d/15)", nav_fail_count_);
                std::vector<Eigen::Vector3d> hover = {pos};
                publishTrajectory(hover);
                return;
            } else {
                // 长期卡死：切换到逃脱模式找替代方向
                RCLCPP_WARN(this->get_logger(),
                    "Nav stuck %d cycles! Escape mode.", nav_fail_count_);
                nav_fail_count_ = 0;
                // 尝试附近可达点（沿洞穴轴方向）
                bool escaped = false;
                for (double step_try : {2.0, 1.0, 0.5}) {
                    for (double dz : {0.0, 0.5, -0.5, 1.0, -1.0, 1.5}) {
                        for (double dy : {0.0, 1.0, -1.0}) {
                            Eigen::Vector3d esc(
                                pos.x() - step_try,
                                pos.y() + dy,
                                std::max(3.0, std::min(20.0, pos.z() + dz)));
                            std::lock_guard<std::mutex> lock(map_mutex_);
                            if (map_->isPathFree(pos, esc, 0.2)) {
                                path = {pos, esc};
                                escaped = true;
                                RCLCPP_WARN(this->get_logger(),
                                    "Nav escape -> [%.1f,%.1f,%.1f]",
                                    esc.x(), esc.y(), esc.z());
                                break;
                            }
                        }
                        if (escaped) break;
                    }
                    if (escaped) break;
                }
                if (!escaped) {
                    // 无条件上升
                    Eigen::Vector3d up(pos.x(), pos.y(),
                        std::min(20.0, pos.z() + 1.5));
                    path = {pos, up};
                    RCLCPP_WARN(this->get_logger(),
                        "Nav ascent to z=%.1f", up.z());
                }
            }
        }
    }

    current_path_ = path;
    path_index_ = 0;
    nav_fail_count_ = 0;
    publishPathVisualization(path);
    publishTrajectory(path);
}

// ============================================================
// 发布轨迹（MultiDOFJointTrajectory格式）
// ============================================================
void PathPlannerNode::publishTrajectory(const std::vector<Eigen::Vector3d>& path)
{
    if (path.empty()) return;

    // 如果路径只有1个点（目标），在前面插入当前位置，确保controller能获得正确的飞行方向和速度
    std::vector<Eigen::Vector3d> effective_path = path;
    if (effective_path.size() == 1 && has_odom_) {
        Eigen::Vector3d pos;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            pos = current_pos_;
        }
        double dist_to_target = (effective_path[0] - pos).norm();
        if (dist_to_target > 0.5) {
            effective_path.insert(effective_path.begin(), pos);
        }
    }

    trajectory_msgs::msg::MultiDOFJointTrajectory traj_msg;
    traj_msg.header.stamp = this->now();
    traj_msg.header.frame_id = "world";

    // 预警区减速：obstacle_speed_scale_ 由 onPointCloud warn zone 实时更新
    double effective_speed = getEffectiveSpeed() * obstacle_speed_scale_;
    effective_speed = std::max(0.5, effective_speed);  // 最低 0.5 m/s，防止时间戳重叠
    double t = 0.0;

    // 预警区爬升偏置：仅对末尾目标点（1~2点轨迹）施加Z偏移，
    // 避免将整条 A* 多点路径整体上移而压入天花板
    if (obstacle_climb_bias_ > 0.01 && effective_path.size() <= 2) {
        effective_path.back().z() += obstacle_climb_bias_;
        effective_path.back().z() = std::min(effective_path.back().z(), 20.0);  // 不超过20m
    }

    // 预计算每个航点处的转弯角度，用于拐弯减速
    // cos_angles[i] = cos(路径在第i点的转弯角)，1.0=直飞，-1.0=掉头
    std::vector<double> cos_angles(effective_path.size(), 1.0);
    for (size_t i = 1; i + 1 < effective_path.size(); ++i) {
        Eigen::Vector3d d1 = (effective_path[i] - effective_path[i-1]);
        Eigen::Vector3d d2 = (effective_path[i+1] - effective_path[i]);
        if (d1.norm() > 0.01 && d2.norm() > 0.01) {
            cos_angles[i] = d1.normalized().dot(d2.normalized());
        }
    }

    for (size_t i = 0; i < effective_path.size(); ++i) {
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;

        // 位置
        geometry_msgs::msg::Transform transform;
        transform.translation.x = effective_path[i].x();
        transform.translation.y = effective_path[i].y();
        transform.translation.z = effective_path[i].z();
        transform.rotation.w = 1.0;  // 无旋转（保持朝向）

        // 计算朝向（面向下一个航点）
        if (i + 1 < effective_path.size()) {
            Eigen::Vector3d dir = effective_path[i+1] - effective_path[i];
            if (dir.norm() > 0.1) {
                double yaw = std::atan2(dir.y(), dir.x());
                double cy = std::cos(yaw * 0.5);
                double sy = std::sin(yaw * 0.5);
                transform.rotation.z = sy;
                transform.rotation.w = cy;
            }
        }

        point.transforms.push_back(transform);

        // 速度：在拐弯处减速
        // 查看当前航点和下一个航点的转弯角，取较小的减速因子
        geometry_msgs::msg::Twist velocity;
        if (i + 1 < effective_path.size()) {
            Eigen::Vector3d dir = effective_path[i+1] - effective_path[i];
            double dist = dir.norm();
            if (dist > 0.01) {
                dir.normalize();
                // [改进] 减速因子：cos_angle < 0.8 (>37° turn) → 线性降速（更早启动减速）
                // cos = 1.0 → factor = 1.0 (全速)
                // cos = 0.5 → factor ≈ 0.4
                // cos = 0.0 → factor = 0.2 (最低20%速度，更保守)
                // cos < 0.0 → factor = 0.2 (大角度转弯极慢)
                double corner_cos = std::min(cos_angles[i], 
                    (i + 1 < cos_angles.size()) ? cos_angles[i+1] : 1.0);
                double speed_factor = 1.0;
                if (corner_cos < 0.8) {  // 从0.7改为0.8，更早开始减速
                    // 最低速度从0.3降至0.2
                    speed_factor = std::max(0.2, 0.2 + 0.8 * (corner_cos / 0.8));
                }
                // 距离也限制速度：短段减速，避免快速震荡
                if (dist < 2.5) {  // 从2.0改为2.5
                    speed_factor = std::min(speed_factor, std::max(0.2, dist / 2.5));
                }
                double seg_speed = effective_speed * speed_factor;
                velocity.linear.x = dir.x() * seg_speed;
                velocity.linear.y = dir.y() * seg_speed;
                velocity.linear.z = dir.z() * seg_speed;
            }
        }
        point.velocities.push_back(velocity);

        // 加速度（零）
        geometry_msgs::msg::Twist acceleration;
        point.accelerations.push_back(acceleration);

        // 时间戳（使用段内实际速度计算时间）
        if (i > 0) {
            double seg_dist = (effective_path[i] - effective_path[i-1]).norm();
            // 用该段的减速速度计算时间（与上面速度计算保持一致）
            double corner_cos_prev = std::min(cos_angles[i-1],
                (i < cos_angles.size()) ? cos_angles[i] : 1.0);
            double sf = 1.0;
            if (corner_cos_prev < 0.8) {  // 与上面减速逻辑一致
                sf = std::max(0.2, 0.2 + 0.8 * (corner_cos_prev / 0.8));
            }
            if (seg_dist < 2.5) {
                sf = std::min(sf, std::max(0.2, seg_dist / 2.5));
            }
            t += seg_dist / (effective_speed * sf);
        }
        point.time_from_start.sec = static_cast<int32_t>(t);
        point.time_from_start.nanosec = static_cast<uint32_t>(
            (t - point.time_from_start.sec) * 1e9);

        traj_msg.points.push_back(point);
    }

    pub_trajectory_->publish(traj_msg);
}

// ============================================================
// 发布路径（用于可视化）
// ============================================================
void PathPlannerNode::publishPath(const std::vector<Eigen::Vector3d>& path)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "world";

    for (const auto& pt : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = pt.x();
        pose.pose.position.y = pt.y();
        pose.pose.position.z = pt.z();
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    pub_path_->publish(path_msg);
}

// ============================================================
// 发布节点健康状态
// ============================================================
void PathPlannerNode::publishNodeHealth()
{
    state_machine::msg::Answer msg;
    msg.node_name = node_name_in_sm_;
    msg.state = 1;  // RUNNING
    msg.info = "path_planner running";
    msg.timestamp = this->now();
    pub_health_->publish(msg);
}

// ============================================================
// 发布地图可视化
// ============================================================
void PathPlannerNode::publishMapVisualization()
{
    std::lock_guard<std::mutex> lock(map_mutex_);

    Eigen::Vector3d pos;
    {
        std::lock_guard<std::mutex> lock2(odom_mutex_);
        pos = current_pos_;
    }

    auto grid = map_->toOccupancyGrid2D(pos.z() - 2.0, pos.z() + 2.0);
    grid.header.stamp = this->now();
    grid.header.frame_id = "world";
    pub_map_->publish(grid);
}

// ============================================================
// 发布路径可视化（Marker）
// ============================================================
void PathPlannerNode::publishPathVisualization(const std::vector<Eigen::Vector3d>& path)
{
    visualization_msgs::msg::MarkerArray markers;

    // 路径线条
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.stamp = this->now();
    line_marker.header.frame_id = "world";
    line_marker.ns = "path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.2;
    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;
    line_marker.lifetime.sec = 5;

    for (const auto& pt : path) {
        geometry_msgs::msg::Point p;
        p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
        line_marker.points.push_back(p);
    }
    markers.markers.push_back(line_marker);

    // 航点球体
    for (size_t i = 0; i < path.size(); ++i) {
        visualization_msgs::msg::Marker sphere;
        sphere.header = line_marker.header;
        sphere.ns = "waypoints";
        sphere.id = static_cast<int>(i);
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = path[i].x();
        sphere.pose.position.y = path[i].y();
        sphere.pose.position.z = path[i].z();
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.5;
        sphere.color.r = 1.0;
        sphere.color.g = 0.5;
        sphere.color.b = 0.0;
        sphere.color.a = 0.8;
        sphere.lifetime.sec = 5;
        markers.markers.push_back(sphere);
    }

    pub_markers_->publish(markers);

    // 同时发布nav_msgs::Path
    publishPath(path);
}

// ============================================================
// 辅助：将PointCloud2转换为Eigen向量列表
// ============================================================
std::vector<Eigen::Vector3d> PathPlannerNode::pointCloud2ToEigen(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
    std::vector<Eigen::Vector3d> points;

    try {
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        points.reserve(msg->width * msg->height);

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x, y = *iter_y, z = *iter_z;
            // 过滤NaN和无穷大
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            if (x == 0.0f && y == 0.0f && z == 0.0f) continue;
            points.emplace_back(x, y, z);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "PointCloud2 parsing error: %s", e.what());
    }

    return points;
}

// ============================================================
// 辅助：检查是否到达目标
// ============================================================
bool PathPlannerNode::hasReachedGoal(const Eigen::Vector3d& goal, double tol) const
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return (current_pos_ - goal).norm() < tol;
}

// ============================================================
// 辅助：选择最佳frontier点
// ============================================================
Eigen::Vector3d PathPlannerNode::selectBestFrontier(
    const std::vector<Eigen::Vector3d>& frontiers) const
{
    if (frontiers.empty()) return current_pos_;

    Eigen::Vector3d pos;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        pos = current_pos_;
    }

    // 策略1：强烈偏好 Z方向差 < 3m 的frontier（可达性优先）
    double best_score = -std::numeric_limits<double>::max();
    Eigen::Vector3d best_frontier = frontiers[0];

    for (const auto& f : frontiers) {
        double dist = (f - pos).norm();
        double dz = std::abs(f.z() - pos.z());

        // 过滤太近的frontier
        if (dist < 2.0) continue;

        // 高度差超过 4m 的 frontier 严重降分（洞穴内不可达）
        double dz_penalty = (dz > 4.0) ? -50.0 : (dz > 2.0 ? -dz * 2.0 : 0.0);

        // 评分：偏好中等距离（5-15米）
        double dist_score = -std::abs(dist - 8.0) / 8.0;
        // 高度偏好：贴近当前高度而非固定 explore_height
        double height_score = -dz / 3.0;

        double score = dist_score + height_score + dz_penalty;

        if (score > best_score) {
            best_score = score;
            best_frontier = f;
        }
    }

    return best_frontier;
}

}  // namespace path_planner

// ============================================================
// main函数
// ============================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_planner::PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
