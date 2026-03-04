#include "path_planner_pkg/path_planner_node.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <queue>

namespace path_planner {

// 26-连通邻居方向（3D空间中的所有相邻格子）
const std::vector<std::tuple<int,int,int>> AStarPlanner::NEIGHBORS_26 = {
    // 面邻居（6个）
    {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1},
    // 边邻居（12个）
    {1,1,0}, {1,-1,0}, {-1,1,0}, {-1,-1,0},
    {1,0,1}, {1,0,-1}, {-1,0,1}, {-1,0,-1},
    {0,1,1}, {0,1,-1}, {0,-1,1}, {0,-1,-1},
    // 角邻居（8个）
    {1,1,1}, {1,1,-1}, {1,-1,1}, {1,-1,-1},
    {-1,1,1}, {-1,1,-1}, {-1,-1,1}, {-1,-1,-1}
};

AStarPlanner::AStarPlanner(const OccupancyMap3D& map, double robot_radius)
    : map_(map), robot_radius_(robot_radius)
{}

int AStarPlanner::nodeIndex(int x, int y, int z) const
{
    return x + y * map_.sizeX() + z * map_.sizeX() * map_.sizeY();
}

double AStarPlanner::heuristic(int x1, int y1, int z1,
                                int x2, int y2, int z2) const
{
    // 使用欧氏距离作为启发函数（乘以分辨率转换为实际距离）
    double dx = (x2 - x1) * map_.resolution();
    double dy = (y2 - y1) * map_.resolution();
    double dz = (z2 - z1) * map_.resolution();
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::vector<Eigen::Vector3d> AStarPlanner::plan(const Eigen::Vector3d& start,
                                                   const Eigen::Vector3d& goal)
{
    // 将世界坐标转换为栅格坐标
    int sx, sy, sz, gx, gy, gz;
    if (!map_.worldToGrid(start.x(), start.y(), start.z(), sx, sy, sz)) {
        return {};
    }
    if (!map_.worldToGrid(goal.x(), goal.y(), goal.z(), gx, gy, gz)) {
        return {};
    }

    // 如果起点或终点在障碍物中，尝试找最近的自由格子
    if (!map_.isFreeGrid(sx, sy, sz)) {
        // 在起点附近搜索自由格子
        bool found = false;
        for (int r = 1; r <= 5 && !found; ++r) {
            for (int dz = -r; dz <= r && !found; ++dz) {
                for (int dy = -r; dy <= r && !found; ++dy) {
                    for (int dx = -r; dx <= r && !found; ++dx) {
                        if (map_.isFreeGrid(sx+dx, sy+dy, sz+dz)) {
                            sx += dx; sy += dy; sz += dz;
                            found = true;
                        }
                    }
                }
            }
        }
        if (!found) return {};
    }

    // 优先队列（最小堆）
    using NodePQ = std::priority_queue<Node3D, std::vector<Node3D>, std::greater<Node3D>>;
    NodePQ open_set;

    // 记录已访问节点和来源
    std::unordered_map<int, Node3D> came_from;
    std::unordered_map<int, double> g_score;

    int start_idx = nodeIndex(sx, sy, sz);
    int goal_idx  = nodeIndex(gx, gy, gz);

    Node3D start_node;
    start_node.x = sx; start_node.y = sy; start_node.z = sz;
    start_node.g_cost = 0.0;
    start_node.h_cost = heuristic(sx, sy, sz, gx, gy, gz);
    start_node.parent_x = sx; start_node.parent_y = sy; start_node.parent_z = sz;

    open_set.push(start_node);
    g_score[start_idx] = 0.0;
    came_from[start_idx] = start_node;

    int inflate = static_cast<int>(std::ceil(robot_radius_ / map_.resolution()));
    int max_iter = 100000;
    int iter = 0;

    while (!open_set.empty() && iter++ < max_iter) {
        Node3D current = open_set.top();
        open_set.pop();

        int current_idx = nodeIndex(current.x, current.y, current.z);

        // 到达目标
        if (current_idx == goal_idx ||
            (std::abs(current.x - gx) <= 1 &&
             std::abs(current.y - gy) <= 1 &&
             std::abs(current.z - gz) <= 1)) {
            return reconstructPath(came_from, current);
        }

        // 展开邻居
        for (const auto& [dx, dy, dz] : NEIGHBORS_26) {
            int nx = current.x + dx;
            int ny = current.y + dy;
            int nz = current.z + dz;

            // 边界检查
            if (nx < 0 || nx >= map_.sizeX() ||
                ny < 0 || ny >= map_.sizeY() ||
                nz < 0 || nz >= map_.sizeZ()) {
                continue;
            }

            // 碰撞检查（球形膨胀，避免正方体膨胀在对角方向过度/不足）
            bool collision = false;
            for (int ix = -inflate; ix <= inflate && !collision; ++ix) {
                for (int iy = -inflate; iy <= inflate && !collision; ++iy) {
                    for (int iz = -inflate; iz <= inflate && !collision; ++iz) {
                        // 只检查球内的格子（欧氏距离 <= robot_radius）
                        if (ix*ix + iy*iy + iz*iz > inflate*inflate) continue;
                        int8_t val = map_.getCell(nx+ix, ny+iy, nz+iz);
                        if (val == 100 || val == 50) collision = true;  // 100=障碍物, 50=预膨胀
                    }
                }
            }
            if (collision) continue;

            // 计算移动代价（对角移动代价更高）
            double move_cost = std::sqrt(dx*dx + dy*dy + dz*dz) * map_.resolution();

            // 对未知区域增加代价（鼓励在已知区域规划）
            int8_t cell_val = map_.getCell(nx, ny, nz);
            if (cell_val == -1) {
                move_cost *= 2.0;  // 未知区域代价翻倍
            }

            double tentative_g = g_score[current_idx] + move_cost;
            int neighbor_idx = nodeIndex(nx, ny, nz);

            if (g_score.find(neighbor_idx) == g_score.end() ||
                tentative_g < g_score[neighbor_idx]) {

                g_score[neighbor_idx] = tentative_g;

                Node3D neighbor;
                neighbor.x = nx; neighbor.y = ny; neighbor.z = nz;
                neighbor.g_cost = tentative_g;
                neighbor.h_cost = heuristic(nx, ny, nz, gx, gy, gz);
                neighbor.parent_x = current.x;
                neighbor.parent_y = current.y;
                neighbor.parent_z = current.z;

                came_from[neighbor_idx] = neighbor;
                open_set.push(neighbor);
            }
        }
    }

    // 未找到路径
    return {};
}

std::vector<Eigen::Vector3d> AStarPlanner::reconstructPath(
    const std::unordered_map<int, Node3D>& came_from,
    const Node3D& current) const
{
    std::vector<Eigen::Vector3d> path;
    Node3D node = current;

    int max_steps = 10000;
    int steps = 0;

    while (steps++ < max_steps) {
        double wx, wy, wz;
        map_.gridToWorld(node.x, node.y, node.z, wx, wy, wz);
        path.emplace_back(wx, wy, wz);

        int parent_idx = nodeIndex(node.parent_x, node.parent_y, node.parent_z);
        int current_idx = nodeIndex(node.x, node.y, node.z);

        if (parent_idx == current_idx) break;  // 到达起点

        auto it = came_from.find(parent_idx);
        if (it == came_from.end()) break;
        node = it->second;
    }

    std::reverse(path.begin(), path.end());

    // 路径平滑：去除冗余中间点
    if (path.size() > 2) {
        std::vector<Eigen::Vector3d> smoothed;
        smoothed.push_back(path.front());

        for (size_t i = 1; i < path.size() - 1; ++i) {
            // 如果从上一个保留点到下下个点是无碰撞的，跳过当前点
            if (!map_.isPathFree(smoothed.back(), path[i+1], robot_radius_)) {
                smoothed.push_back(path[i]);
            }
        }
        smoothed.push_back(path.back());

        // ---- 拐角安全膨胀：在尖锐拐弯处插入偏移航点 ----
        // 当连续两段的夹角 < 150°（cos < 0.87）时，控制器跟踪会切弯，
        // 在凸出障碍物旁尤其危险。此处在拐点处沿角平分线外侧偏移
        // 一定距离插入额外航点，迫使无人机绕弯飞行。
        // [改进] 根据转弯角度动态调整偏移距离：
        //   - 角度 > 90° (cos < 0)：偏移 2.5 倍半径
        //   - 角度 > 60° (cos < 0.5)：偏移 2.0 倍半径
        //   - 角度 > 30° (cos < 0.87)：偏移 1.5 倍半径
        if (smoothed.size() >= 3) {
            std::vector<Eigen::Vector3d> cornered;
            cornered.push_back(smoothed.front());

            for (size_t i = 1; i < smoothed.size() - 1; ++i) {
                Eigen::Vector3d d1 = (smoothed[i] - smoothed[i-1]).normalized();
                Eigen::Vector3d d2 = (smoothed[i+1] - smoothed[i]).normalized();
                double cos_angle = d1.dot(d2);

                if (cos_angle < 0.87) {  // 从0.7改为0.87，更早检测转弯
                    // 拐弯（< 150°）：插入偏移航点
                    // 角平分线外侧方向 = -(d1 + d2).normalized()
                    Eigen::Vector3d bisect = -(d1 + d2);
                    double bn = bisect.norm();
                    if (bn > 1e-6) {
                        bisect /= bn;
                        // [改进] 根据转弯角度动态计算偏移距离
                        double offset_mult = 1.5;
                        if (cos_angle < 0.0) {
                            offset_mult = 2.5;  // 超过90°转弯
                        } else if (cos_angle < 0.5) {
                            offset_mult = 2.0;  // 超过60°转弯
                        }
                        double offset_dist = robot_radius_ * offset_mult;
                        Eigen::Vector3d offset_pt = smoothed[i] + bisect * offset_dist;
                        // 只有偏移点可通行时才插入
                        if (map_.isPathFree(smoothed[i-1], offset_pt, robot_radius_ * 0.5) &&
                            map_.isPathFree(offset_pt, smoothed[i+1], robot_radius_ * 0.5)) {
                            cornered.push_back(offset_pt);
                        }
                    }
                }
                cornered.push_back(smoothed[i]);
            }
            cornered.push_back(smoothed.back());
            return cornered;
        }

        return smoothed;
    }

    return path;
}

}  // namespace path_planner
