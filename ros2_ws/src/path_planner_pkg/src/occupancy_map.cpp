#include "path_planner_pkg/path_planner_node.hpp"
#include <cmath>
#include <algorithm>

namespace path_planner {

// ============================================================
// OccupancyMap3D 实现
// ============================================================

OccupancyMap3D::OccupancyMap3D(double resolution,
                                double origin_x, double origin_y, double origin_z,
                                int size_x, int size_y, int size_z)
    : resolution_(resolution),
      origin_x_(origin_x), origin_y_(origin_y), origin_z_(origin_z),
      size_x_(size_x), size_y_(size_y), size_z_(size_z)
{
    // 初始化所有格子为未知（-1）
    data_.assign(size_x_ * size_y_ * size_z_, -1);
}

bool OccupancyMap3D::worldToGrid(double wx, double wy, double wz,
                                  int& gx, int& gy, int& gz) const
{
    gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    gz = static_cast<int>(std::floor((wz - origin_z_) / resolution_));
    return inBounds(gx, gy, gz);
}

void OccupancyMap3D::gridToWorld(int gx, int gy, int gz,
                                  double& wx, double& wy, double& wz) const
{
    wx = origin_x_ + (gx + 0.5) * resolution_;
    wy = origin_y_ + (gy + 0.5) * resolution_;
    wz = origin_z_ + (gz + 0.5) * resolution_;
}

void OccupancyMap3D::setCell(int gx, int gy, int gz, int8_t value)
{
    if (inBounds(gx, gy, gz)) {
        data_[index(gx, gy, gz)] = value;
    }
}

int8_t OccupancyMap3D::getCell(int gx, int gy, int gz) const
{
    if (!inBounds(gx, gy, gz)) return -1;
    return data_[index(gx, gy, gz)];
}

void OccupancyMap3D::updateFromPointCloud(const std::vector<Eigen::Vector3d>& points,
                                           const Eigen::Vector3d& sensor_origin,
                                           double max_range)
{
    int sx, sy, sz;
    if (!worldToGrid(sensor_origin.x(), sensor_origin.y(), sensor_origin.z(),
                     sx, sy, sz)) {
        return;
    }

    for (const auto& pt : points) {
        double dx = pt.x() - sensor_origin.x();
        double dy = pt.y() - sensor_origin.y();
        double dz = pt.z() - sensor_origin.z();
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < 0.1 || dist > max_range) continue;

        // 标记终点为占用
        int ex, ey, ez;
        if (worldToGrid(pt.x(), pt.y(), pt.z(), ex, ey, ez)) {
            setCell(ex, ey, ez, 100);
        }

        // 用Bresenham 3D光线投射标记自由空间
        // 简化版：沿射线方向均匀采样
        int steps = static_cast<int>(dist / resolution_) - 1;
        for (int i = 1; i <= steps; ++i) {
            double t = static_cast<double>(i) / (steps + 1);
            double fx = sensor_origin.x() + t * dx;
            double fy = sensor_origin.y() + t * dy;
            double fz = sensor_origin.z() + t * dz;
            int gx, gy, gz;
            if (worldToGrid(fx, fy, fz, gx, gy, gz)) {
                int8_t current_val = getCell(gx, gy, gz);
                // [修复] 光线穿过的格子应该被清除为自由，
                // 包括之前误标记为占用的格子（消除虞障碍物）
                if (current_val == -1 || current_val == 100) {
                    setCell(gx, gy, gz, 0);
                }
            }
        }
    }
}

bool OccupancyMap3D::isFree(double wx, double wy, double wz) const
{
    int gx, gy, gz;
    if (!worldToGrid(wx, wy, wz, gx, gy, gz)) return false;
    return isFreeGrid(gx, gy, gz);
}

bool OccupancyMap3D::isFreeGrid(int gx, int gy, int gz) const
{
    int8_t val = getCell(gx, gy, gz);
    // 允许通过明确自由和未知区域（只阻挡确认的障碍物）
    // 之前只有 val==0 才可通行，导致A*在未探索洞穴中无法规划
    return val != 100;  // 0(自由) 和 -1(未知) 都可通行
}

bool OccupancyMap3D::isPathFree(const Eigen::Vector3d& from,
                                  const Eigen::Vector3d& to,
                                  double robot_radius) const
{
    Eigen::Vector3d dir = to - from;
    double dist = dir.norm();
    if (dist < 1e-6) return true;

    dir.normalize();
    int steps = static_cast<int>(dist / (resolution_ * 0.5)) + 1;
    int inflate = static_cast<int>(std::ceil(robot_radius / resolution_));

    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        Eigen::Vector3d pt = from + t * (to - from);

        int gx, gy, gz;
        if (!worldToGrid(pt.x(), pt.y(), pt.z(), gx, gy, gz)) return false;

        // 检查膨胀后的区域
        for (int dx = -inflate; dx <= inflate; ++dx) {
            for (int dy = -inflate; dy <= inflate; ++dy) {
                for (int dz = -inflate; dz <= inflate; ++dz) {
                    int nx = gx + dx, ny = gy + dy, nz = gz + dz;
                    if (inBounds(nx, ny, nz)) {
                        int8_t val = getCell(nx, ny, nz);
                        if (val == 100) return false;  // 碰到障碍物
                    }
                }
            }
        }
    }
    return true;
}

std::vector<Eigen::Vector3d> OccupancyMap3D::getFrontierPoints() const
{
    std::vector<Eigen::Vector3d> frontiers;

    // 全地图搜索，step=1 确保不遗漏薄壁 frontier
    int step = 1;
    int max_frontiers = 500;
    
    for (int z = 1; z < size_z_ - 1; z += step) {
        for (int y = 1; y < size_y_ - 1; y += step) {
            for (int x = 1; x < size_x_ - 1; x += step) {
                if (getCell(x, y, z) != 0) continue;

                bool has_unknown = false;
                for (int dz = -1; dz <= 1 && !has_unknown; ++dz) {
                    for (int dy = -1; dy <= 1 && !has_unknown; ++dy) {
                        for (int dx = -1; dx <= 1 && !has_unknown; ++dx) {
                            if (dx == 0 && dy == 0 && dz == 0) continue;
                            if (getCell(x+dx, y+dy, z+dz) == -1) {
                                has_unknown = true;
                            }
                        }
                    }
                }

                if (has_unknown) {
                    double wx, wy, wz;
                    gridToWorld(x, y, z, wx, wy, wz);
                    frontiers.emplace_back(wx, wy, wz);
                    if (static_cast<int>(frontiers.size()) >= max_frontiers) {
                        return frontiers;
                    }
                }
            }
        }
    }

    return frontiers;
}

std::vector<Eigen::Vector3d> OccupancyMap3D::getFrontierPointsNear(
    const Eigen::Vector3d& robot_pos, double search_radius) const
{
    std::vector<Eigen::Vector3d> frontiers;

    // 将机器人位置转换为栅格坐标
    int cx, cy, cz;
    if (!worldToGrid(robot_pos.x(), robot_pos.y(), robot_pos.z(), cx, cy, cz)) {
        return frontiers;
    }

    // 计算搜索范围（栅格单位）
    int r = static_cast<int>(std::ceil(search_radius / resolution_));
    int x_min = std::max(1, cx - r);
    int x_max = std::min(size_x_ - 2, cx + r);
    int y_min = std::max(1, cy - r);
    int y_max = std::min(size_y_ - 2, cy + r);
    int z_min = std::max(1, cz - r);
    int z_max = std::min(size_z_ - 2, cz + r);

    int max_frontiers = 500;

    // step=1 逐格搜索，确保不遗漏
    for (int z = z_min; z <= z_max; ++z) {
        for (int y = y_min; y <= y_max; ++y) {
            for (int x = x_min; x <= x_max; ++x) {
                if (getCell(x, y, z) != 0) continue;

                bool has_unknown = false;
                for (int dz = -1; dz <= 1 && !has_unknown; ++dz) {
                    for (int dy = -1; dy <= 1 && !has_unknown; ++dy) {
                        for (int dx = -1; dx <= 1 && !has_unknown; ++dx) {
                            if (dx == 0 && dy == 0 && dz == 0) continue;
                            if (getCell(x+dx, y+dy, z+dz) == -1) {
                                has_unknown = true;
                            }
                        }
                    }
                }

                if (has_unknown) {
                    double wx, wy, wz;
                    gridToWorld(x, y, z, wx, wy, wz);
                    frontiers.emplace_back(wx, wy, wz);
                    if (static_cast<int>(frontiers.size()) >= max_frontiers) {
                        return frontiers;
                    }
                }
            }
        }
    }

    return frontiers;
}

nav_msgs::msg::OccupancyGrid OccupancyMap3D::toOccupancyGrid2D(
    double z_min, double z_max) const
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = size_x_;
    grid.info.height = size_y_;
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.assign(size_x_ * size_y_, -1);

    int gz_min, gz_max;
    {
        int dummy;
        worldToGrid(0, 0, z_min, dummy, dummy, gz_min);
        worldToGrid(0, 0, z_max, dummy, dummy, gz_max);
        gz_min = std::max(0, gz_min);
        gz_max = std::min(size_z_ - 1, gz_max);
    }

    for (int y = 0; y < size_y_; ++y) {
        for (int x = 0; x < size_x_; ++x) {
            int8_t best = -1;
            for (int z = gz_min; z <= gz_max; ++z) {
                int8_t val = getCell(x, y, z);
                if (val == 100) { best = 100; break; }
                if (val == 0 && best == -1) best = 0;
            }
            grid.data[x + y * size_x_] = best;
        }
    }

    return grid;
}

}  // namespace path_planner
