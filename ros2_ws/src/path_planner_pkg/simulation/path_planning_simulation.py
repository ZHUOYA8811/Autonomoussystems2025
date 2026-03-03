#!/usr/bin/env python3
"""
UAV 洞穴探索路径规划仿真
========================
本脚本实现并可视化用于UAV洞穴探索的路径规划算法：
  - A* 搜索算法（3D占用栅格地图）
  - RRT 算法（快速随机树探索）

用于验证算法正确性，无需ROS2环境即可运行。

使用方法：
  python3 path_planning_simulation.py
  python3 path_planning_simulation.py --algorithm rrt
  python3 path_planning_simulation.py --algorithm both
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import heapq
import random
import time
import argparse
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
from collections import defaultdict
import math


# ============================================================
# 洞穴环境生成
# ============================================================

class CaveEnvironment:
    """
    模拟洞穴环境的3D占用栅格地图
    
    洞穴由一系列圆柱形隧道和球形空腔组成，
    包含4个目标物体（灯笼）供UAV寻找。
    """
    
    def __init__(self, resolution: float = 0.5):
        """
        初始化洞穴环境
        
        Args:
            resolution: 栅格分辨率（米/格）
        """
        self.resolution = resolution
        
        # 地图范围（米）
        self.x_range = (-60, 10)
        self.y_range = (-5, 25)
        self.z_range = (0, 20)
        
        # 计算栅格尺寸
        self.nx = int((self.x_range[1] - self.x_range[0]) / resolution)
        self.ny = int((self.y_range[1] - self.y_range[0]) / resolution)
        self.nz = int((self.z_range[1] - self.z_range[0]) / resolution)
        
        # 占用栅格：0=自由, 1=占用, -1=未知
        self.grid = np.full((self.nx, self.ny, self.nz), -1, dtype=np.int8)
        
        # 目标物体位置（灯笼）
        self.lanterns = [
            np.array([-10.0, 10.0, 5.0]),
            np.array([-25.0, 8.0, 6.0]),
            np.array([-40.0, 12.0, 5.0]),
            np.array([-50.0, 10.0, 7.0]),
        ]
        
        # UAV起始位置
        self.start_pos = np.array([-38.0, 10.0, 8.0])
        
        # 洞穴入口
        self.cave_entrance = np.array([-5.0, 10.0, 5.0])
        
        # 生成洞穴结构
        self._generate_cave()
        
    def _generate_cave(self):
        """生成洞穴地图结构"""
        # 首先将所有格子标记为占用（实心岩石）
        self.grid[:] = 1
        
        # 挖掘主隧道（沿X轴方向）
        self._carve_tunnel(
            start=(-55, 10, 5),
            end=(5, 10, 5),
            radius=3.5
        )
        
        # 挖掘分支隧道1（向上分叉）
        self._carve_tunnel(
            start=(-20, 10, 5),
            end=(-20, 20, 8),
            radius=2.5
        )
        
        # 挖掘分支隧道2（向下分叉）
        self._carve_tunnel(
            start=(-35, 10, 5),
            end=(-35, 2, 4),
            radius=2.0
        )
        
        # 挖掘空腔1（宽阔区域）
        self._carve_sphere(center=(-15, 10, 5), radius=5.0)
        
        # 挖掘空腔2
        self._carve_sphere(center=(-40, 10, 6), radius=4.5)
        
        # 挖掘空腔3（洞穴深处）
        self._carve_sphere(center=(-52, 10, 6), radius=4.0)
        
        # 添加一些随机障碍物（石柱）
        np.random.seed(42)
        for _ in range(8):
            cx = np.random.uniform(-50, -5)
            cy = np.random.uniform(7, 13)
            cz = np.random.uniform(3, 7)
            # 只在自由空间内添加石柱
            if self.is_free_world(cx, cy, cz):
                self._add_pillar(center=(cx, cy, cz), radius=0.8, height=2.0)
        
        print(f"Cave generated: {self.nx}x{self.ny}x{self.nz} grid "
              f"({self.nx*self.resolution:.0f}x{self.ny*self.resolution:.0f}x"
              f"{self.nz*self.resolution:.0f}m)")
        free_count = np.sum(self.grid == 0)
        total = self.nx * self.ny * self.nz
        print(f"Free space: {free_count}/{total} cells "
              f"({100*free_count/total:.1f}%)")
    
    def _carve_tunnel(self, start: tuple, end: tuple, radius: float):
        """沿直线挖掘圆柱形隧道"""
        start = np.array(start)
        end = np.array(end)
        direction = end - start
        length = np.linalg.norm(direction)
        if length < 1e-6:
            return
        direction = direction / length
        
        steps = int(length / (self.resolution * 0.5)) + 1
        for i in range(steps + 1):
            t = i / steps
            center = start + t * (end - start)
            self._carve_sphere(center=tuple(center), radius=radius)
    
    def _carve_sphere(self, center: tuple, radius: float):
        """挖掘球形空间"""
        cx, cy, cz = center
        r_cells = int(radius / self.resolution) + 1
        
        gx, gy, gz = self.world_to_grid(cx, cy, cz)
        
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                for dz in range(-r_cells, r_cells + 1):
                    nx, ny, nz = gx + dx, gy + dy, gz + dz
                    if not self.in_bounds(nx, ny, nz):
                        continue
                    wx, wy, wz = self.grid_to_world(nx, ny, nz)
                    dist = math.sqrt((wx-cx)**2 + (wy-cy)**2 + (wz-cz)**2)
                    if dist <= radius:
                        self.grid[nx, ny, nz] = 0  # 标记为自由
    
    def _add_pillar(self, center: tuple, radius: float, height: float):
        """添加圆柱形石柱障碍物"""
        cx, cy, cz = center
        r_cells = int(radius / self.resolution) + 1
        h_cells = int(height / self.resolution) + 1
        
        gx, gy, gz = self.world_to_grid(cx, cy, cz)
        
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                for dz in range(0, h_cells + 1):
                    nx, ny, nz = gx + dx, gy + dy, gz + dz
                    if not self.in_bounds(nx, ny, nz):
                        continue
                    wx, wy, wz = self.grid_to_world(nx, ny, nz)
                    dist_2d = math.sqrt((wx-cx)**2 + (wy-cy)**2)
                    if dist_2d <= radius:
                        self.grid[nx, ny, nz] = 1  # 标记为占用
    
    def world_to_grid(self, wx: float, wy: float, wz: float) -> Tuple[int, int, int]:
        """世界坐标 -> 栅格索引"""
        gx = int((wx - self.x_range[0]) / self.resolution)
        gy = int((wy - self.y_range[0]) / self.resolution)
        gz = int((wz - self.z_range[0]) / self.resolution)
        return gx, gy, gz
    
    def grid_to_world(self, gx: int, gy: int, gz: int) -> Tuple[float, float, float]:
        """栅格索引 -> 世界坐标（格子中心）"""
        wx = self.x_range[0] + (gx + 0.5) * self.resolution
        wy = self.y_range[0] + (gy + 0.5) * self.resolution
        wz = self.z_range[0] + (gz + 0.5) * self.resolution
        return wx, wy, wz
    
    def in_bounds(self, gx: int, gy: int, gz: int) -> bool:
        """检查栅格索引是否在边界内"""
        return (0 <= gx < self.nx and
                0 <= gy < self.ny and
                0 <= gz < self.nz)
    
    def is_free_grid(self, gx: int, gy: int, gz: int) -> bool:
        """检查栅格格子是否自由"""
        if not self.in_bounds(gx, gy, gz):
            return False
        return self.grid[gx, gy, gz] == 0
    
    def is_free_world(self, wx: float, wy: float, wz: float) -> bool:
        """检查世界坐标点是否在自由空间"""
        gx, gy, gz = self.world_to_grid(wx, wy, wz)
        return self.is_free_grid(gx, gy, gz)
    
    def is_path_free(self, p1: np.ndarray, p2: np.ndarray,
                      robot_radius: float = 0.5) -> bool:
        """检查两点之间的路径是否无碰撞"""
        direction = p2 - p1
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            return True
        
        steps = max(int(dist / (self.resolution * 0.5)), 2)
        inflate = max(1, int(robot_radius / self.resolution))
        
        for i in range(steps + 1):
            t = i / steps
            pt = p1 + t * direction
            gx, gy, gz = self.world_to_grid(*pt)
            
            # 检查膨胀区域
            for dx in range(-inflate, inflate + 1):
                for dy in range(-inflate, inflate + 1):
                    for dz in range(-inflate, inflate + 1):
                        nx, ny, nz = gx + dx, gy + dy, gz + dz
                        if self.in_bounds(nx, ny, nz):
                            if self.grid[nx, ny, nz] == 1:
                                return False
        return True
    
    def get_frontier_points(self) -> List[np.ndarray]:
        """获取frontier点（自由空间与未知空间的边界）"""
        frontiers = []
        for gz in range(1, self.nz - 1):
            for gy in range(1, self.ny - 1):
                for gx in range(1, self.nx - 1):
                    if self.grid[gx, gy, gz] != 0:
                        continue
                    # 检查26-邻居中是否有未知格子
                    has_unknown = False
                    for dz in range(-1, 2):
                        for dy in range(-1, 2):
                            for dx in range(-1, 2):
                                if dx == dy == dz == 0:
                                    continue
                                if self.grid[gx+dx, gy+dy, gz+dz] == -1:
                                    has_unknown = True
                                    break
                            if has_unknown:
                                break
                        if has_unknown:
                            break
                    if has_unknown:
                        wx, wy, wz = self.grid_to_world(gx, gy, gz)
                        frontiers.append(np.array([wx, wy, wz]))
        return frontiers
    
    def simulate_sensor_update(self, uav_pos: np.ndarray, sensor_range: float = 8.0):
        """
        模拟传感器更新：将UAV周围的未知区域标记为已知
        （在真实系统中由深度相机完成）
        """
        gx0, gy0, gz0 = self.world_to_grid(*uav_pos)
        r_cells = int(sensor_range / self.resolution)
        
        updated = 0
        for dz in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                for dx in range(-r_cells, r_cells + 1):
                    nx, ny, nz = gx0 + dx, gy0 + dy, gz0 + dz
                    if not self.in_bounds(nx, ny, nz):
                        continue
                    wx, wy, wz = self.grid_to_world(nx, ny, nz)
                    dist = math.sqrt((wx - uav_pos[0])**2 +
                                     (wy - uav_pos[1])**2 +
                                     (wz - uav_pos[2])**2)
                    if dist <= sensor_range:
                        # 标记为已知（不改变占用状态，只是"揭示"地图）
                        # 在真实系统中，这里会根据深度图更新
                        updated += 1
        return updated


# ============================================================
# A* 路径规划算法
# ============================================================

class AStarPlanner3D:
    """
    3D A* 路径规划算法
    
    在3D占用栅格地图上搜索从起点到终点的最优路径。
    使用欧氏距离作为启发函数，支持26-连通邻居。
    """
    
    # 26-连通邻居方向
    NEIGHBORS_26 = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    ]
    
    def __init__(self, env: CaveEnvironment, robot_radius: float = 0.5):
        """
        初始化A*规划器
        
        Args:
            env: 洞穴环境对象
            robot_radius: 机器人碰撞半径（米）
        """
        self.env = env
        self.robot_radius = robot_radius
        self.inflate = max(1, int(robot_radius / env.resolution))
    
    def heuristic(self, gx1: int, gy1: int, gz1: int,
                   gx2: int, gy2: int, gz2: int) -> float:
        """欧氏距离启发函数"""
        dx = (gx2 - gx1) * self.env.resolution
        dy = (gy2 - gy1) * self.env.resolution
        dz = (gz2 - gz1) * self.env.resolution
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def is_collision_free(self, gx: int, gy: int, gz: int) -> bool:
        """检查格子是否无碰撞（考虑机器人半径）"""
        for dx in range(-self.inflate, self.inflate + 1):
            for dy in range(-self.inflate, self.inflate + 1):
                for dz in range(-self.inflate, self.inflate + 1):
                    nx, ny, nz = gx + dx, gy + dy, gz + dz
                    if self.env.in_bounds(nx, ny, nz):
                        if self.env.grid[nx, ny, nz] == 1:
                            return False
        return True
    
    def plan(self, start: np.ndarray, goal: np.ndarray) -> List[np.ndarray]:
        """
        规划从start到goal的路径
        
        Args:
            start: 起点世界坐标 [x, y, z]
            goal: 终点世界坐标 [x, y, z]
            
        Returns:
            路径点列表（世界坐标），空列表表示规划失败
        """
        t_start = time.time()
        
        # 转换为栅格坐标
        sx, sy, sz = self.env.world_to_grid(*start)
        gx, gy, gz = self.env.world_to_grid(*goal)
        
        # 边界检查
        if not self.env.in_bounds(sx, sy, sz):
            print(f"  [A*] Start position out of bounds: {start}")
            return []
        if not self.env.in_bounds(gx, gy, gz):
            print(f"  [A*] Goal position out of bounds: {goal}")
            return []
        
        # 如果起点在障碍物中，寻找最近自由格子
        if not self.is_collision_free(sx, sy, sz):
            found = False
            for r in range(1, 6):
                for dz in range(-r, r+1):
                    for dy in range(-r, r+1):
                        for dx in range(-r, r+1):
                            if self.is_collision_free(sx+dx, sy+dy, sz+dz):
                                sx, sy, sz = sx+dx, sy+dy, sz+dz
                                found = True
                                break
                        if found: break
                    if found: break
                if found: break
            if not found:
                print(f"  [A*] Cannot find free cell near start")
                return []
        
        # 优先队列：(f_cost, g_cost, x, y, z)
        open_heap = []
        h0 = self.heuristic(sx, sy, sz, gx, gy, gz)
        heapq.heappush(open_heap, (h0, 0.0, sx, sy, sz))
        
        # 记录代价和来源
        g_score = {(sx, sy, sz): 0.0}
        came_from = {}
        closed_set = set()
        
        nodes_expanded = 0
        max_iter = 200000
        
        while open_heap and nodes_expanded < max_iter:
            f, g, cx, cy, cz = heapq.heappop(open_heap)
            
            if (cx, cy, cz) in closed_set:
                continue
            closed_set.add((cx, cy, cz))
            nodes_expanded += 1
            
            # 到达目标（允许1格误差）
            if (abs(cx - gx) <= 1 and abs(cy - gy) <= 1 and abs(cz - gz) <= 1):
                path = self._reconstruct_path(came_from, (cx, cy, cz), (sx, sy, sz))
                t_end = time.time()
                print(f"  [A*] Path found: {len(path)} waypoints, "
                      f"{nodes_expanded} nodes expanded, "
                      f"{t_end - t_start:.3f}s")
                return self._smooth_path(path)
            
            # 展开邻居
            for dx, dy, dz in self.NEIGHBORS_26:
                nx, ny, nz = cx + dx, cy + dy, cz + dz
                
                if not self.env.in_bounds(nx, ny, nz):
                    continue
                if (nx, ny, nz) in closed_set:
                    continue
                if not self.is_collision_free(nx, ny, nz):
                    continue
                
                # 移动代价（对角移动更贵）
                move_cost = math.sqrt(dx*dx + dy*dy + dz*dz) * self.env.resolution
                
                # 对未知区域增加代价
                if self.env.grid[nx, ny, nz] == -1:
                    move_cost *= 1.5
                
                tentative_g = g + move_cost
                
                if (nx, ny, nz) not in g_score or tentative_g < g_score[(nx, ny, nz)]:
                    g_score[(nx, ny, nz)] = tentative_g
                    h = self.heuristic(nx, ny, nz, gx, gy, gz)
                    heapq.heappush(open_heap, (tentative_g + h, tentative_g, nx, ny, nz))
                    came_from[(nx, ny, nz)] = (cx, cy, cz)
        
        t_end = time.time()
        print(f"  [A*] Path NOT found after {nodes_expanded} nodes, {t_end - t_start:.3f}s")
        return []
    
    def _reconstruct_path(self, came_from: dict,
                           current: tuple, start: tuple) -> List[np.ndarray]:
        """从came_from字典重建路径"""
        path = []
        node = current
        
        while node != start and node in came_from:
            wx, wy, wz = self.env.grid_to_world(*node)
            path.append(np.array([wx, wy, wz]))
            node = came_from[node]
        
        # 添加起点
        wx, wy, wz = self.env.grid_to_world(*start)
        path.append(np.array([wx, wy, wz]))
        path.reverse()
        return path
    
    def _smooth_path(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """路径平滑：去除冗余中间点"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # 尝试跳过中间点
            j = len(path) - 1
            while j > i + 1:
                if self.env.is_path_free(smoothed[-1], path[j], self.robot_radius):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j
        
        return smoothed


# ============================================================
# RRT 路径规划算法
# ============================================================

class RRTPlanner3D:
    """
    3D RRT (Rapidly-exploring Random Tree) 路径规划算法
    
    通过随机采样快速探索未知空间，适合复杂洞穴环境。
    支持goal-biased采样以提高收敛速度。
    """
    
    @dataclass
    class Node:
        pos: np.ndarray
        parent: int = -1
        
    def __init__(self, env: CaveEnvironment,
                  step_size: float = 1.5,
                  goal_bias: float = 0.1,
                  max_iter: int = 10000,
                  robot_radius: float = 0.5):
        """
        初始化RRT规划器
        
        Args:
            env: 洞穴环境对象
            step_size: 每步扩展距离（米）
            goal_bias: 直接采样目标点的概率
            max_iter: 最大迭代次数
            robot_radius: 机器人碰撞半径（米）
        """
        self.env = env
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.max_iter = max_iter
        self.robot_radius = robot_radius
        self.tree: List[RRTPlanner3D.Node] = []
        
    def plan(self, start: np.ndarray, goal: np.ndarray) -> List[np.ndarray]:
        """
        规划从start到goal的路径
        
        Args:
            start: 起点世界坐标
            goal: 终点世界坐标
            
        Returns:
            路径点列表，空列表表示规划失败
        """
        t_start = time.time()
        
        self.tree = [self.Node(pos=start.copy(), parent=-1)]
        
        # 地图边界
        x_min, x_max = self.env.x_range
        y_min, y_max = self.env.y_range
        z_min, z_max = self.env.z_range
        
        for iteration in range(self.max_iter):
            # 采样随机点
            if random.random() < self.goal_bias:
                sample = goal.copy()
            else:
                sample = np.array([
                    random.uniform(x_min, x_max),
                    random.uniform(y_min, y_max),
                    random.uniform(z_min, z_max)
                ])
            
            # 找最近节点
            nearest_idx = self._nearest_node(sample)
            nearest_pos = self.tree[nearest_idx].pos
            
            # 向采样点延伸
            new_pos = self._steer(nearest_pos, sample)
            
            # 碰撞检测
            if not self.env.is_path_free(nearest_pos, new_pos, self.robot_radius):
                continue
            
            # 添加新节点
            new_idx = len(self.tree)
            self.tree.append(self.Node(pos=new_pos.copy(), parent=nearest_idx))
            
            # 检查是否到达目标
            dist_to_goal = np.linalg.norm(new_pos - goal)
            if dist_to_goal < self.step_size * 1.5:
                if self.env.is_path_free(new_pos, goal, self.robot_radius):
                    goal_idx = len(self.tree)
                    self.tree.append(self.Node(pos=goal.copy(), parent=new_idx))
                    path = self._extract_path(goal_idx)
                    t_end = time.time()
                    print(f"  [RRT] Path found: {len(path)} waypoints, "
                          f"{iteration+1} iterations, "
                          f"{t_end - t_start:.3f}s")
                    return path
        
        t_end = time.time()
        print(f"  [RRT] Path NOT found after {self.max_iter} iterations, "
              f"{t_end - t_start:.3f}s")
        
        # 返回到最近节点的路径
        best_idx = self._nearest_node(goal)
        return self._extract_path(best_idx)
    
    def _nearest_node(self, point: np.ndarray) -> int:
        """找树中距离给定点最近的节点"""
        min_dist = float('inf')
        nearest = 0
        for i, node in enumerate(self.tree):
            d = np.linalg.norm(node.pos - point)
            if d < min_dist:
                min_dist = d
                nearest = i
        return nearest
    
    def _steer(self, from_pos: np.ndarray, to_pos: np.ndarray) -> np.ndarray:
        """从from_pos向to_pos延伸step_size距离"""
        direction = to_pos - from_pos
        dist = np.linalg.norm(direction)
        if dist <= self.step_size:
            return to_pos.copy()
        return from_pos + (direction / dist) * self.step_size
    
    def _extract_path(self, goal_idx: int) -> List[np.ndarray]:
        """从树中提取路径"""
        path = []
        idx = goal_idx
        while idx >= 0:
            path.append(self.tree[idx].pos.copy())
            idx = self.tree[idx].parent
        path.reverse()
        return path
    
    def get_tree_edges(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """获取所有树边（用于可视化）"""
        edges = []
        for i, node in enumerate(self.tree):
            if node.parent >= 0:
                edges.append((self.tree[node.parent].pos, node.pos))
        return edges


# ============================================================
# 探索仿真
# ============================================================

class ExplorationSimulation:
    """
    UAV洞穴探索仿真
    
    模拟UAV在洞穴中自主探索的过程：
    1. 从起点起飞
    2. 飞向洞穴入口
    3. 在洞穴内自主探索，寻找所有灯笼
    4. 返回起点
    """
    
    def __init__(self, env: CaveEnvironment, algorithm: str = 'astar'):
        """
        初始化探索仿真
        
        Args:
            env: 洞穴环境
            algorithm: 使用的算法 ('astar', 'rrt', 'both')
        """
        self.env = env
        self.algorithm = algorithm
        
        # 创建规划器
        self.astar = AStarPlanner3D(env, robot_radius=0.5)
        self.rrt = RRTPlanner3D(env, step_size=1.5, goal_bias=0.15,
                                  max_iter=8000, robot_radius=0.5)
        
        # 仿真状态
        self.uav_pos = env.start_pos.copy()
        self.trajectory = [self.uav_pos.copy()]
        self.found_lanterns = []
        self.planned_paths = []  # 存储所有规划的路径
        self.rrt_trees = []      # 存储RRT树（用于可视化）
        
        # 统计信息
        self.total_distance = 0.0
        self.planning_times = []
        self.num_replans = 0
        
    def run(self) -> Dict:
        """
        运行完整的探索仿真
        
        Returns:
            包含仿真结果的字典
        """
        print("\n" + "="*60)
        print("UAV 洞穴探索仿真")
        print(f"算法: {self.algorithm.upper()}")
        print("="*60)
        
        # 阶段1：飞向洞穴入口
        print("\n[阶段1] 飞向洞穴入口...")
        path1 = self._plan_path(self.uav_pos, self.env.cave_entrance)
        if path1:
            self._execute_path(path1)
            self.planned_paths.append(('entrance', path1))
        
        # 阶段2：在洞穴内探索
        print("\n[阶段2] 开始洞穴探索...")
        self._explore_cave()
        
        # 阶段3：返回起点
        print("\n[阶段3] 返回起点...")
        path_home = self._plan_path(self.uav_pos, self.env.start_pos)
        if path_home:
            self._execute_path(path_home)
            self.planned_paths.append(('home', path_home))
        
        # 统计结果
        results = {
            'total_distance': self.total_distance,
            'found_lanterns': len(self.found_lanterns),
            'total_lanterns': len(self.env.lanterns),
            'num_replans': self.num_replans,
            'trajectory_length': len(self.trajectory),
            'planning_times': self.planning_times,
        }
        
        print("\n" + "="*60)
        print("仿真完成！")
        print(f"  总飞行距离: {self.total_distance:.1f} 米")
        print(f"  发现灯笼: {len(self.found_lanterns)}/{len(self.env.lanterns)}")
        print(f"  重规划次数: {self.num_replans}")
        if self.planning_times:
            print(f"  平均规划时间: {np.mean(self.planning_times)*1000:.1f} ms")
        print("="*60)
        
        return results
    
    def _plan_path(self, start: np.ndarray, goal: np.ndarray) -> List[np.ndarray]:
        """使用选定算法规划路径"""
        t0 = time.time()
        
        if self.algorithm == 'astar':
            path = self.astar.plan(start, goal)
        elif self.algorithm == 'rrt':
            path = self.rrt.plan(start, goal)
            if self.rrt.tree:
                self.rrt_trees.append(self.rrt.get_tree_edges())
        else:  # both: 先用A*，失败则用RRT
            path = self.astar.plan(start, goal)
            if not path:
                print("  A* failed, trying RRT...")
                path = self.rrt.plan(start, goal)
                if self.rrt.tree:
                    self.rrt_trees.append(self.rrt.get_tree_edges())
        
        t1 = time.time()
        self.planning_times.append(t1 - t0)
        self.num_replans += 1
        
        return path
    
    def _execute_path(self, path: List[np.ndarray]):
        """模拟UAV沿路径飞行"""
        for waypoint in path[1:]:  # 跳过起点
            # 记录轨迹（简化：直接跳到航点）
            self.total_distance += np.linalg.norm(waypoint - self.uav_pos)
            self.uav_pos = waypoint.copy()
            self.trajectory.append(self.uav_pos.copy())
            
            # 检测附近的灯笼
            self._check_lantern_detection()
    
    def _check_lantern_detection(self, detection_range: float = 5.0):
        """检查是否发现附近的灯笼"""
        for i, lantern in enumerate(self.env.lanterns):
            if i in [l[0] for l in self.found_lanterns]:
                continue  # 已发现
            dist = np.linalg.norm(self.uav_pos - lantern)
            if dist < detection_range:
                self.found_lanterns.append((i, lantern.copy(), self.uav_pos.copy()))
                print(f"  *** 发现灯笼 #{i+1} at {lantern} "
                      f"(距离: {dist:.1f}m) ***")
    
    def _explore_cave(self):
        """在洞穴内进行frontier-based探索"""
        max_exploration_steps = 20
        step = 0
        
        # 预定义探索航点（模拟frontier探索）
        # 在真实系统中，这些会由frontier检测动态生成
        exploration_waypoints = [
            np.array([-10.0, 10.0, 5.0]),
            np.array([-20.0, 10.0, 5.0]),
            np.array([-25.0, 8.0, 6.0]),
            np.array([-30.0, 10.0, 5.0]),
            np.array([-35.0, 10.0, 5.0]),
            np.array([-40.0, 12.0, 5.0]),
            np.array([-45.0, 10.0, 6.0]),
            np.array([-50.0, 10.0, 7.0]),
            np.array([-53.0, 10.0, 6.0]),
        ]
        
        for waypoint in exploration_waypoints:
            if step >= max_exploration_steps:
                break
            
            print(f"  探索航点 {step+1}: {waypoint}")
            path = self._plan_path(self.uav_pos, waypoint)
            
            if path:
                self._execute_path(path)
                self.planned_paths.append((f'explore_{step}', path))
            
            step += 1
            
            # 如果发现所有灯笼，停止探索
            if len(self.found_lanterns) >= len(self.env.lanterns):
                print("  所有灯笼已找到！停止探索。")
                break


# ============================================================
# 可视化
# ============================================================

def visualize_results(env: CaveEnvironment,
                       sim: ExplorationSimulation,
                       save_path: str = None):
    """
    可视化仿真结果
    
    生成多个图表展示：
    1. 3D路径规划结果
    2. 2D俯视图（XY平面）
    3. 2D侧视图（XZ平面）
    4. 算法性能统计
    """
    fig = plt.figure(figsize=(20, 16))
    fig.suptitle('UAV 洞穴探索路径规划仿真\n'
                 f'算法: {sim.algorithm.upper()} | '
                 f'发现灯笼: {len(sim.found_lanterns)}/{len(env.lanterns)} | '
                 f'总距离: {sim.total_distance:.1f}m',
                 fontsize=14, fontweight='bold')
    
    # ---- 图1：3D视图 ----
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    _plot_3d_view(ax1, env, sim)
    
    # ---- 图2：XY俯视图 ----
    ax2 = fig.add_subplot(2, 2, 2)
    _plot_top_view(ax2, env, sim)
    
    # ---- 图3：XZ侧视图 ----
    ax3 = fig.add_subplot(2, 2, 3)
    _plot_side_view(ax3, env, sim)
    
    # ---- 图4：性能统计 ----
    ax4 = fig.add_subplot(2, 2, 4)
    _plot_statistics(ax4, env, sim)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n图像已保存至: {save_path}")
    
    plt.show()


def _plot_3d_view(ax, env: CaveEnvironment, sim: ExplorationSimulation):
    """绘制3D视图"""
    ax.set_title('3D 路径规划视图', fontsize=11)
    
    # 绘制洞穴障碍物（采样显示）
    occupied = np.argwhere(env.grid == 1)
    if len(occupied) > 0:
        # 随机采样以减少绘制数量
        idx = np.random.choice(len(occupied),
                                min(2000, len(occupied)), replace=False)
        pts = occupied[idx]
        wx = env.x_range[0] + (pts[:, 0] + 0.5) * env.resolution
        wy = env.y_range[0] + (pts[:, 1] + 0.5) * env.resolution
        wz = env.z_range[0] + (pts[:, 2] + 0.5) * env.resolution
        ax.scatter(wx, wy, wz, c='gray', alpha=0.1, s=1, label='障碍物')
    
    # 绘制UAV轨迹
    traj = np.array(sim.trajectory)
    if len(traj) > 1:
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2],
                'b-', linewidth=2, alpha=0.8, label='UAV轨迹')
    
    # 绘制规划路径
    colors = plt.cm.Set2(np.linspace(0, 1, len(sim.planned_paths)))
    for i, (name, path) in enumerate(sim.planned_paths):
        if len(path) > 1:
            pts = np.array(path)
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
                    '--', color=colors[i], linewidth=1.5, alpha=0.7)
    
    # 绘制灯笼
    for i, lantern in enumerate(env.lanterns):
        found = any(f[0] == i for f in sim.found_lanterns)
        color = 'gold' if found else 'red'
        marker = '*' if found else 'o'
        ax.scatter(*lantern, c=color, s=200, marker=marker, zorder=5,
                   label=f'灯笼{i+1}{"(已找到)" if found else ""}')
    
    # 绘制起点和终点
    ax.scatter(*env.start_pos, c='green', s=200, marker='^',
               zorder=5, label='起点')
    ax.scatter(*env.cave_entrance, c='purple', s=150, marker='D',
               zorder=5, label='洞穴入口')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend(loc='upper right', fontsize=7)


def _plot_top_view(ax, env: CaveEnvironment, sim: ExplorationSimulation):
    """绘制XY俯视图"""
    ax.set_title('XY 俯视图', fontsize=11)
    
    # 绘制2D占用栅格（Z方向投影）
    z_mid = int(env.nz * 0.4)  # 取中间高度层
    grid_2d = env.grid[:, :, z_mid].T
    
    # 颜色映射
    cmap = plt.cm.colors.ListedColormap(['white', 'lightgray', 'darkgray'])
    bounds = [-1.5, -0.5, 0.5, 1.5]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)
    
    extent = [env.x_range[0], env.x_range[1],
              env.y_range[0], env.y_range[1]]
    ax.imshow(grid_2d, extent=extent, origin='lower',
              cmap=cmap, norm=norm, alpha=0.6, aspect='auto')
    
    # 绘制轨迹
    traj = np.array(sim.trajectory)
    if len(traj) > 1:
        ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=2,
                alpha=0.8, label='UAV轨迹', zorder=3)
    
    # 绘制规划路径
    colors = plt.cm.Set2(np.linspace(0, 1, max(1, len(sim.planned_paths))))
    for i, (name, path) in enumerate(sim.planned_paths):
        if len(path) > 1:
            pts = np.array(path)
            ax.plot(pts[:, 0], pts[:, 1], '--',
                    color=colors[i % len(colors)], linewidth=1.5,
                    alpha=0.7, zorder=2)
            # 绘制航点
            ax.scatter(pts[:, 0], pts[:, 1], s=20,
                       color=colors[i % len(colors)], zorder=4)
    
    # 绘制灯笼
    for i, lantern in enumerate(env.lanterns):
        found = any(f[0] == i for f in sim.found_lanterns)
        ax.scatter(lantern[0], lantern[1],
                   c='gold' if found else 'red',
                   s=200, marker='*', zorder=5,
                   edgecolors='black', linewidths=0.5)
        ax.annotate(f'L{i+1}', (lantern[0], lantern[1]),
                    textcoords='offset points', xytext=(5, 5),
                    fontsize=8, fontweight='bold')
    
    # 起点和入口
    ax.scatter(*env.start_pos[:2], c='green', s=200, marker='^',
               zorder=5, label='起点', edgecolors='black')
    ax.scatter(*env.cave_entrance[:2], c='purple', s=150, marker='D',
               zorder=5, label='洞穴入口', edgecolors='black')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(env.x_range)
    ax.set_ylim(env.y_range)


def _plot_side_view(ax, env: CaveEnvironment, sim: ExplorationSimulation):
    """绘制XZ侧视图"""
    ax.set_title('XZ 侧视图', fontsize=11)
    
    # 绘制2D占用栅格（Y方向投影）
    y_mid = int(env.ny * 0.5)
    grid_2d = env.grid[:, y_mid, :].T
    
    cmap = plt.cm.colors.ListedColormap(['white', 'lightgray', 'darkgray'])
    bounds = [-1.5, -0.5, 0.5, 1.5]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)
    
    extent = [env.x_range[0], env.x_range[1],
              env.z_range[0], env.z_range[1]]
    ax.imshow(grid_2d, extent=extent, origin='lower',
              cmap=cmap, norm=norm, alpha=0.6, aspect='auto')
    
    # 绘制轨迹
    traj = np.array(sim.trajectory)
    if len(traj) > 1:
        ax.plot(traj[:, 0], traj[:, 2], 'b-', linewidth=2,
                alpha=0.8, label='UAV轨迹', zorder=3)
    
    # 绘制规划路径
    colors = plt.cm.Set2(np.linspace(0, 1, max(1, len(sim.planned_paths))))
    for i, (name, path) in enumerate(sim.planned_paths):
        if len(path) > 1:
            pts = np.array(path)
            ax.plot(pts[:, 0], pts[:, 2], '--',
                    color=colors[i % len(colors)], linewidth=1.5,
                    alpha=0.7, zorder=2)
    
    # 绘制灯笼
    for i, lantern in enumerate(env.lanterns):
        found = any(f[0] == i for f in sim.found_lanterns)
        ax.scatter(lantern[0], lantern[2],
                   c='gold' if found else 'red',
                   s=200, marker='*', zorder=5,
                   edgecolors='black', linewidths=0.5)
    
    ax.scatter(env.start_pos[0], env.start_pos[2], c='green',
               s=200, marker='^', zorder=5, label='起点')
    ax.scatter(env.cave_entrance[0], env.cave_entrance[2], c='purple',
               s=150, marker='D', zorder=5, label='洞穴入口')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(env.x_range)
    ax.set_ylim(env.z_range)


def _plot_statistics(ax, env: CaveEnvironment, sim: ExplorationSimulation):
    """绘制性能统计图"""
    ax.set_title('算法性能统计', fontsize=11)
    ax.axis('off')
    
    # 统计数据
    stats = [
        ('算法', sim.algorithm.upper()),
        ('总飞行距离', f'{sim.total_distance:.1f} m'),
        ('发现灯笼', f'{len(sim.found_lanterns)}/{len(env.lanterns)}'),
        ('规划次数', str(sim.num_replans)),
        ('轨迹点数', str(len(sim.trajectory))),
    ]
    
    if sim.planning_times:
        stats.extend([
            ('平均规划时间', f'{np.mean(sim.planning_times)*1000:.1f} ms'),
            ('最大规划时间', f'{np.max(sim.planning_times)*1000:.1f} ms'),
            ('最小规划时间', f'{np.min(sim.planning_times)*1000:.1f} ms'),
        ])
    
    # 绘制表格
    y = 0.95
    for key, val in stats:
        ax.text(0.1, y, f'{key}:', transform=ax.transAxes,
                fontsize=11, fontweight='bold', va='top')
        ax.text(0.55, y, val, transform=ax.transAxes,
                fontsize=11, va='top')
        y -= 0.1
    
    # 绘制规划时间条形图
    if sim.planning_times:
        ax_inset = ax.inset_axes([0.05, 0.05, 0.9, 0.35])
        times_ms = [t * 1000 for t in sim.planning_times]
        bars = ax_inset.bar(range(len(times_ms)), times_ms,
                             color='steelblue', alpha=0.7)
        ax_inset.set_xlabel('规划次数', fontsize=8)
        ax_inset.set_ylabel('时间 (ms)', fontsize=8)
        ax_inset.set_title('各次规划耗时', fontsize=9)
        ax_inset.tick_params(labelsize=7)
        ax_inset.axhline(y=np.mean(times_ms), color='red',
                          linestyle='--', linewidth=1, label='平均')
        ax_inset.legend(fontsize=7)


def compare_algorithms(env: CaveEnvironment):
    """
    比较A*和RRT算法的性能
    """
    print("\n" + "="*60)
    print("算法比较：A* vs RRT")
    print("="*60)
    
    results = {}
    
    for algo in ['astar', 'rrt']:
        print(f"\n运行 {algo.upper()} 算法...")
        
        # 重置环境（使用相同的随机种子）
        env_copy = CaveEnvironment(resolution=env.resolution)
        sim = ExplorationSimulation(env_copy, algorithm=algo)
        result = sim.run()
        results[algo] = (sim, result)
    
    # 绘制比较图
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    fig.suptitle('A* vs RRT 算法比较', fontsize=14, fontweight='bold')
    
    for i, (algo, (sim, result)) in enumerate(results.items()):
        ax = axes[i]
        ax.set_title(f'{algo.upper()} 算法\n'
                     f'距离: {result["total_distance"]:.1f}m | '
                     f'灯笼: {result["found_lanterns"]}/{result["total_lanterns"]}',
                     fontsize=11)
        
        _plot_top_view(ax, sim.env, sim)
    
    plt.tight_layout()
    plt.savefig('/home/ubuntu/path_planner_pkg/simulation/algorithm_comparison.png',
                dpi=150, bbox_inches='tight')
    plt.show()
    
    # 打印比较表格
    print("\n" + "="*60)
    print("性能比较结果")
    print("="*60)
    print(f"{'指标':<25} {'A*':>15} {'RRT':>15}")
    print("-"*55)
    
    metrics = [
        ('总飞行距离 (m)', 'total_distance', '{:.1f}'),
        ('发现灯笼数', 'found_lanterns', '{}'),
        ('规划次数', 'num_replans', '{}'),
    ]
    
    for name, key, fmt in metrics:
        a_val = fmt.format(results['astar'][1][key])
        r_val = fmt.format(results['rrt'][1][key])
        print(f"{name:<25} {a_val:>15} {r_val:>15}")
    
    if results['astar'][1]['planning_times'] and results['rrt'][1]['planning_times']:
        a_time = np.mean(results['astar'][1]['planning_times']) * 1000
        r_time = np.mean(results['rrt'][1]['planning_times']) * 1000
        print(f"{'平均规划时间 (ms)':<25} {a_time:>15.1f} {r_time:>15.1f}")
    
    print("="*60)
    
    return results


# ============================================================
# 主函数
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='UAV洞穴探索路径规划仿真')
    parser.add_argument('--algorithm', type=str, default='both',
                        choices=['astar', 'rrt', 'both', 'compare'],
                        help='路径规划算法选择')
    parser.add_argument('--resolution', type=float, default=0.5,
                        help='地图分辨率（米）')
    parser.add_argument('--save', type=str, default=None,
                        help='保存图像路径')
    args = parser.parse_args()
    
    print("初始化洞穴环境...")
    env = CaveEnvironment(resolution=args.resolution)
    
    if args.algorithm == 'compare':
        # 比较两种算法
        compare_algorithms(env)
    else:
        # 运行单一算法仿真
        sim = ExplorationSimulation(env, algorithm=args.algorithm)
        results = sim.run()
        
        # 可视化
        save_path = args.save or \
            f'/home/ubuntu/path_planner_pkg/simulation/simulation_{args.algorithm}.png'
        visualize_results(env, sim, save_path=save_path)


if __name__ == '__main__':
    main()
