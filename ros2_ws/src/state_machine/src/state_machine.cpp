#include "state_machine/state_machine.hpp"
#include <cmath>

namespace drone_mission {

StateMachine::StateMachine() : Node("state_machine_node") {
    // 1. 获取监控节点列表
    this->declare_parameter<std::vector<std::string>>("monitored_node_list", std::vector<std::string>{"controller", "sampler"});
    this->declare_parameter<double>("takeoff_height", 5.0);
    this->declare_parameter<double>("checkpoint_reach_dist_m", 0.3);

    auto node_list = this->get_parameter("monitored_node_list").as_string_array();
    for (const auto& name : node_list) {
        monitored_nodes_.push_back({name});
    }

    checkpoint_reach_dist_m_ = this->get_parameter("checkpoint_reach_dist_m").as_double();

    // 2. 初始化发布者
    pub_state_ = this->create_publisher<std_msgs::msg::String>("statemachine/state", 10);
    pub_cmd_ = this->create_publisher<state_machine::msg::Command>("statemachine/cmd", 10);
    
    // 3. 初始化订阅者
    sub_node_health_ = this->create_subscription<state_machine::msg::Answer>(
        "statemachine/node_health", 10, std::bind(&StateMachine::handleAnswer, this, std::placeholders::_1));
    
    sub_current_state_est_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "current_state_est", 10, std::bind(&StateMachine::onCurrentStateEst, this, std::placeholders::_1));

    // 4. 设置主逻辑定时器 (10Hz)
    periodic_task_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&StateMachine::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "=== State machine started. Current mode: WAITING ===");
}

void StateMachine::onTimer() {
    // 发布当前状态字符串供调试
    auto state_msg = std_msgs::msg::String();
    state_msg.data = toString(current_mission_state_);
    pub_state_->publish(state_msg);

    checkSystemstate();  // 监控系统健康状况
    checkCheckpoints();  // 检查航点到达情况
    handleFlagEvents();  // 处理状态切换逻辑
}

/**
 * @brief 监控所有模块的存活状态
 */
void StateMachine::checkSystemstate() {
    auto now = this->now();
    bool all_nodes_ready = true;

    for (auto& node : monitored_nodes_) {
        if (node.last_node_health.nanoseconds() == 0) {
            node.is_alive = false;
        } else {
            double elapsed_sec = (now - node.last_node_health).seconds();
            node.is_alive = (elapsed_sec <= alive_tol_sec_);
        }
        
        if (!node.is_alive) {
            all_nodes_ready = false;
        }
    }

    if (current_mission_state_ == MissionStates::WAITING && all_nodes_ready) {
        changeState(MissionStates::TAKEOFF, "All monitored modules are ready");
    }

    if (current_mission_state_ != MissionStates::WAITING && 
        current_mission_state_ != MissionStates::DONE && 
        !all_nodes_ready) 
    {
        changeState(MissionStates::ERROR, "Critical node offline, entering emergency mode");
    }
}

/**
 * @brief 处理信号
 */
void StateMachine::handleAnswer(const state_machine::msg::Answer::SharedPtr msg) {
    if (!msg) return;
    auto it = std::find_if(monitored_nodes_.begin(), monitored_nodes_.end(),
                           [&](const NodeInfo& n) { return n.node_name == msg->node_name; });
    
    if (it != monitored_nodes_.end()) {
        it->last_node_health = this->now();
        it->last_state = static_cast<AnswerStates>(msg->state);
    }
}

/**
 * @brief 检查是否到达当前航点 (3D 欧式距离)
 */
void StateMachine::checkCheckpoints() {
    // 如果没有任务点或没收到定位，不执行检查
    if (active_checkpoint_positions_m_.empty() || !has_received_current_pose_) {
        return;
    }

    // 越界保护
    if (active_checkpoint_index_ < 0 || 
        active_checkpoint_index_ >= static_cast<short>(active_checkpoint_positions_m_.size())) {
        return;
    }

    // 获取当前目标航点
    const auto& target = active_checkpoint_positions_m_[active_checkpoint_index_];

    // 计算 3D 距离
    double dx = current_position_m_.x - target.x(); 
    double dy = current_position_m_.y - target.y();
    double dz = current_position_m_.z - target.z();
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    // 如果进入容差范围，触发事件并指向下一个点
    if (distance <= checkpoint_reach_dist_m_) {
        is_checkpoint_reached_ = true;
        active_checkpoint_index_++;
        RCLCPP_INFO(this->get_logger(), "Waypoint reached! Index: %d, Distance remaining: %.2f", active_checkpoint_index_-1, distance);
    }
}

/**
 * @brief 根据“航点到达”等事件处理状态机跳转
 */
void StateMachine::handleFlagEvents() {
    if (is_checkpoint_reached_) {
        int reached_id = active_checkpoint_index_ - 1;

        // 如果在起飞阶段到达了第 0 个点（高度点）
        if (current_mission_state_ == MissionStates::TAKEOFF && reached_id == 0) {
            changeState(MissionStates::TRAVELLING, "Takeoff altitude reached");
        }
        // 以后可以在这里增加：如果到达了巡航点，切换到 EXPLORING 等逻辑

        is_checkpoint_reached_ = false; // 重置信号
    }
}

/**
 * @brief 状态切换与指令分发中心
 */
void StateMachine::changeState(MissionStates target, const std::string& reason) {
    if (current_mission_state_ == target) return;
    
    current_mission_state_ = target;
    logEvent("[State Transition] -> " + toString(target) + " Reason: " + reason);

    switch (current_mission_state_) {
        case MissionStates::WAITING:
            sendCommand("controller", Commands::HOLD);
            break;

        case MissionStates::TAKEOFF:
            {
                // 1. 获取用户设定的“相对起飞高度”参数（比如 5.0 米）
                double delta_h = this->get_parameter("takeoff_height").as_double();
                
                active_checkpoint_positions_m_.clear();
                Eigen::Vector3d p;

                // 2. 核心修改：捕捉当前这一秒的真实坐标
                if (has_received_current_pose_) {
                    p.x() = current_position_m_.x; // 这里会存入 -38.00
                    p.y() = current_position_m_.y; // 这里会存入 9.99
                    // 目标高度 = 现在的 6.93 + 设定的 5.0 = 11.93
                    p.z() = current_position_m_.z + delta_h; 
                } else {
                    // 如果没收到坐标（万一），则保底回退
                    p.x() = 0.0; p.y() = 0.0; p.z() = delta_h;
                }

                // 3. 将这个“动态计算”出的点作为第一个打卡点
                active_checkpoint_positions_m_.push_back(p);
                active_checkpoint_index_ = 0;

                RCLCPP_INFO(this->get_logger(), 
                    "Takeoff setpoint locked：Current(%.2f, %.2f, %.2f) -> Target Alt: %.2f", 
                    current_position_m_.x, current_position_m_.y, current_position_m_.z, p.z());
            }
            sendCommand("controller", Commands::START);
            break;

        case MissionStates::TRAVELLING:
            sendCommand("controller", Commands::START);
            // 这里可以添加对 navigator 或 sampler 的指令
            break;

        case MissionStates::LAND:
            {
                geometry_msgs::msg::Point land_pt;
                if (prepareLandingCheckpoint(land_pt)) {
                    sendCommandWithTarget("sampler", Commands::LAND, land_pt);
                }
            }
            break;

        case MissionStates::ERROR:
            sendCommand("controller", Commands::HOLD);
            break;

        default:
            break;
    }
}

// 辅助函数：计算降落坐标
bool StateMachine::prepareLandingCheckpoint(geometry_msgs::msg::Point& landing_target_out) {
    if (!has_received_current_pose_) return false;
    landing_target_out.x = current_position_m_.x;
    landing_target_out.y = current_position_m_.y;
    landing_target_out.z = 0.0; 
    return true;
}

// 辅助函数：消息下发
void StateMachine::sendCommand(std::string receiver, Commands cmd) {
    state_machine::msg::Command cmd_msg;
    cmd_msg.target = receiver;
    cmd_msg.command = static_cast<uint8_t>(cmd);
    pub_cmd_->publish(cmd_msg);
}

void StateMachine::sendCommandWithTarget(const std::string& receiver, Commands cmd, const geometry_msgs::msg::Point& target) {
    state_machine::msg::Command cmd_msg;
    cmd_msg.target = receiver;
    cmd_msg.command = static_cast<uint8_t>(cmd);
    cmd_msg.target_pos = target;
    cmd_msg.has_target = true;
    pub_cmd_->publish(cmd_msg);
}

void StateMachine::logEvent(const std::string& msg) {
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
}

std::string StateMachine::toString(MissionStates s) {
    switch (s) {
        case MissionStates::WAITING:    return "WAITING";
        case MissionStates::TAKEOFF:    return "TAKEOFF";
        case MissionStates::TRAVELLING:  return "TRAVELLING";
        case MissionStates::EXPLORING:   return "EXPLORING";
        case MissionStates::RETURN_HOME: return "RETURN_HOME";
        case MissionStates::LAND:        return "LAND";
        case MissionStates::DONE:        return "DONE";
        case MissionStates::ERROR:       return "ERROR";
        default:                         return "OTHER";
    }
}

std::string StateMachine::toString(Commands c) {
    switch (c) {
        case Commands::TAKEOFF:     return "TAKEOFF_CMD";
        case Commands::START:       return "START_CMD";
        case Commands::HOLD:        return "HOLD_CMD";
        case Commands::RETURN_HOME: return "RTH_CMD";
        case Commands::LAND:        return "LAND_CMD";
        case Commands::ABORT:       return "ABORT_CMD";
        default:                    return "NONE";
    }
}

void StateMachine::onCurrentStateEst(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_m_ = msg->pose.pose.position;
    has_received_current_pose_ = true;
}

StateMachine::~StateMachine() {}

} // namespace drone_mission

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<drone_mission::StateMachine>());
    rclcpp::shutdown();
    return 0;
}