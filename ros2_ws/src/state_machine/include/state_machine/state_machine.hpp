#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "state_machine/msg/command.hpp"
#include "state_machine/msg/answer.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace drone_mission {

// 定义任务状态枚举
enum class MissionStates { 
    WAITING, 
    TAKEOFF, 
    TRAVELLING, 
    EXPLORING, 
    RETURN_HOME, 
    LAND, 
    DONE, 
    ERROR, 
    ABORTED 
};

// 定义指令枚举 (对应 Command.msg)
enum class Commands : uint8_t { 
    NONE = 0, 
    TAKEOFF = 1, 
    START = 2, 
    HOLD = 3, 
    RETURN_HOME = 4, 
    LAND = 5, 
    ABORT = 6 
};

// 定义应答状态枚举 (对应 Answer.msg)
enum class AnswerStates : uint8_t { 
    UNKNOWN = 0, 
    RUNNING = 1, 
    DONE = 2 
};

// 用于监控节点健康的结构体
struct NodeInfo {
    std::string node_name;
    rclcpp::Time last_node_health{0, 0, RCL_ROS_TIME};
    bool is_alive = false;
    AnswerStates last_state = AnswerStates::UNKNOWN;
};

class StateMachine : public rclcpp::Node {
public:
    StateMachine();
    virtual ~StateMachine();

private:
    // --- 核心定时器逻辑 ---
    void onTimer();
    
    // --- 任务逻辑函数 ---
    void checkSystemstate(); // 你要求的命名称：检查系统健康状态
    void checkCheckpoints(); // 检查是否到达预设航点
    void handleFlagEvents(); // 处理各种触发事件（如数够了灯笼）
    void changeState(MissionStates target_mission_state, const std::string& state_change_reason);

    // --- 回调函数 ---
    void handleAnswer(const state_machine::msg::Answer::SharedPtr msg);
    void onCurrentStateEst(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onLanternDetections(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    // --- 动作与命令 ---
    void sendCommand(std::string receiver_node_name, Commands command_enum);
    void sendCommandWithTarget(const std::string& receiver_node_name, Commands command_enum, const geometry_msgs::msg::Point& target_position_m);
    
    // 简化的降落逻辑：直接设置 Z=0
    bool prepareLandingCheckpoint(geometry_msgs::msg::Point& landing_target_out);

    geometry_msgs::msg::PoseArray buildDiscoveredLanternPoseArray() const;
    void logDiscoveredLanternSummary(const std::string& stage_tag) const;

    // --- 日志与可视化辅助 ---
    std::string toString(MissionStates current_state_est);
    std::string toString(Commands active_command);
    void logEvent(const std::string& event_message);
    void publishLanternMonitor();

    // --- ROS 2 句柄 ---
    rclcpp::TimerBase::SharedPtr periodic_task_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
    rclcpp::Publisher<state_machine::msg::Command>::SharedPtr pub_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_discovered_lanterns_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_lantern_monitor_;
    rclcpp::Subscription<state_machine::msg::Answer>::SharedPtr sub_node_health_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_current_state_est_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_lantern_detections_;

    // --- 内部变量 ---
    MissionStates current_mission_state_ = MissionStates::WAITING;
    std::vector<NodeInfo> monitored_nodes_;
    std::vector<Eigen::Vector3d> active_checkpoint_positions_m_;
    std::vector<geometry_msgs::msg::Point> discovered_lanterns_;
    geometry_msgs::msg::Point current_position_m_;
    bool has_received_current_pose_ = false;
    int active_checkpoint_index_ = -1;
    double alive_tol_sec_ = 10.0; // 心跳超时阈值

    size_t latest_lantern_count_ = 0;
    const size_t kRequiredLanternCount = 4;    // 目标达成需要的数量：4

    bool is_checkpoint_reached_ = false;
    double checkpoint_reach_dist_m_ = 0.5;
    
};

} // namespace drone_mission

#endif // STATE_MACHINE_HPP