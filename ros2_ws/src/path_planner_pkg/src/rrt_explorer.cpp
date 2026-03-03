#include "path_planner_pkg/path_planner_node.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace path_planner {

RRTExplorer::RRTExplorer(const OccupancyMap3D& map,
                           double step_size,
                           double goal_bias,
                           int max_iter,
                           double robot_radius)
    : map_(map),
      step_size_(step_size),
      goal_bias_(goal_bias),
      max_iter_(max_iter),
      robot_radius_(robot_radius),
      rng_(std::random_device{}()),
      dist_01_(0.0, 1.0)
{}

std::vector<Eigen::Vector3d> RRTExplorer::planToFrontier(
    const Eigen::Vector3d& start,
    const std::vector<Eigen::Vector3d>& frontiers)
{
    if (frontiers.empty()) return {};

    // 选择最近的frontier作为目标
    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector3d best_frontier = frontiers[0];

    for (const auto& f : frontiers) {
        double d = (f - start).norm();
        if (d > 2.0 && d < min_dist) {  // 至少2米远
            min_dist = d;
            best_frontier = f;
        }
    }

    return planToGoal(start, best_frontier);
}

std::vector<Eigen::Vector3d> RRTExplorer::planToGoal(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal)
{
    tree_.clear();
    tree_.emplace_back(start, -1);  // 根节点，无父节点

    // 地图边界
    double map_min_x = map_.originX();
    double map_min_y = map_.originY();
    double map_min_z = map_.originZ();
    double map_max_x = map_min_x + map_.sizeX() * map_.resolution();
    double map_max_y = map_min_y + map_.sizeY() * map_.resolution();
    double map_max_z = map_min_z + map_.sizeZ() * map_.resolution();

    std::uniform_real_distribution<double> dist_x(map_min_x, map_max_x);
    std::uniform_real_distribution<double> dist_y(map_min_y, map_max_y);
    std::uniform_real_distribution<double> dist_z(map_min_z, map_max_z);

    for (int iter = 0; iter < max_iter_; ++iter) {
        // 采样随机点（有一定概率直接采样目标点）
        Eigen::Vector3d sample;
        if (dist_01_(rng_) < goal_bias_) {
            sample = goal;
        } else {
            sample = Eigen::Vector3d(dist_x(rng_), dist_y(rng_), dist_z(rng_));
        }

        // 找最近节点
        int nearest_idx = nearestNode(sample);
        const Eigen::Vector3d& nearest_pos = tree_[nearest_idx].pos;

        // 向采样点方向延伸一步
        Eigen::Vector3d new_pos = steer(nearest_pos, sample);

        // 碰撞检测
        if (!map_.isPathFree(nearest_pos, new_pos, robot_radius_)) {
            continue;
        }

        // 添加新节点
        int new_idx = static_cast<int>(tree_.size());
        tree_.emplace_back(new_pos, nearest_idx);

        // 检查是否到达目标
        double dist_to_goal = (new_pos - goal).norm();
        if (dist_to_goal < step_size_ * 1.5) {
            // 尝试直接连接到目标
            if (map_.isPathFree(new_pos, goal, robot_radius_)) {
                tree_.emplace_back(goal, new_idx);
                return extractPath(static_cast<int>(tree_.size()) - 1);
            }
        }
    }

    // 未到达目标，返回到最近节点的路径
    int best_idx = nearestNode(goal);
    if (best_idx > 0) {
        return extractPath(best_idx);
    }

    return {};
}

Eigen::Vector3d RRTExplorer::randomSample(const Eigen::Vector3d& goal)
{
    // 此函数已内联到planToGoal中
    (void)goal;
    return Eigen::Vector3d::Zero();
}

int RRTExplorer::nearestNode(const Eigen::Vector3d& point) const
{
    int nearest = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        double d = (tree_[i].pos - point).norm();
        if (d < min_dist) {
            min_dist = d;
            nearest = i;
        }
    }

    return nearest;
}

Eigen::Vector3d RRTExplorer::steer(const Eigen::Vector3d& from,
                                     const Eigen::Vector3d& to) const
{
    Eigen::Vector3d dir = to - from;
    double dist = dir.norm();

    if (dist <= step_size_) {
        return to;
    }

    return from + (dir / dist) * step_size_;
}

std::vector<Eigen::Vector3d> RRTExplorer::extractPath(int goal_idx) const
{
    std::vector<Eigen::Vector3d> path;
    int idx = goal_idx;

    while (idx >= 0) {
        path.push_back(tree_[idx].pos);
        idx = tree_[idx].parent_idx;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace path_planner
