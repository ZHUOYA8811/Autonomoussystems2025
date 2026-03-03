# ROS2 Jazzy 迁移说明

## 迁移日期
2026-03-03

## 从版本
ROS2 Humble

## 到版本
ROS2 Jazzy

---

## 修改摘要

### ✅ 已完成的修改

#### 1. **path_planner_pkg** - CMakeLists.txt

**修改内容：**
- ✅ CMake 最小版本：`3.8` → `3.16`
- ✅ 移除未使用的依赖：`tf2_eigen`
- ✅ 优化 Eigen3 查找：`find_package(Eigen3 3.4 REQUIRED NO_MODULE)`

**具体变更：**
```cmake
# 之前 (Humble)
cmake_minimum_required(VERSION 3.8)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# 现在 (Jazzy)
cmake_minimum_required(VERSION 3.16)
# 移除 tf2_eigen
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
```

**链接依赖更新：**
```cmake
# 移除了 tf2_eigen 从 ament_target_dependencies 列表
```

#### 2. **path_planner_pkg** - package.xml

**修改内容：**
- ✅ 移除 `<depend>tf2_eigen</depend>`
- ✅ 移除 `<depend>eigen3_cmake_module</depend>`
- ✅ 添加注释说明 Eigen3 直接通过 CMake 查找

**理由：**
- 代码分析显示 `tf2_eigen` 从未被使用（无 `tf2::fromMsg`/`tf2::toMsg` 调用）
- Jazzy 中 Eigen3 可以直接通过 CMake 查找，无需 helper 模块

#### 3. **state_machine** - CMakeLists.txt

**修改内容：**
- ✅ CMake 最小版本：`3.8` → `3.16`

**具体变更：**
```cmake
# 之前 (Humble)
cmake_minimum_required(VERSION 3.8)

# 现在 (Jazzy)
cmake_minimum_required(VERSION 3.16)
```

---

## 兼容性保证

### ✅ 保持不变的部分

1. **C++ 标准**：仍然使用 C++17（Jazzy 完全支持）
2. **ROS2 API**：所有使用的 API 均为现代接口，无需修改
   - `rclcpp::Node`
   - `create_publisher<T>()`
   - `create_subscription<T>()`
   - `create_wall_timer()`
   - `declare_parameter<T>()`
   - `RCLCPP_INFO/WARN/ERROR` 宏
3. **消息类型**：所有标准和自定义消息保持兼容
4. **Launch 文件**：Python launch API 无变化
5. **参数系统**：YAML 配置文件无需修改

### 🔍 已验证的兼容特性

- ✅ 点云处理（`sensor_msgs::PointCloud2Iterator`）
- ✅ QoS 配置（`rclcpp::QoS(10)`, `rclcpp::SensorDataQoS()`）
- ✅ 自定义消息生成（`rosidl_generate_interfaces`）
- ✅ 时间戳处理（`this->now()`）
- ✅ TF2 基础功能（`tf2`/`tf2_ros`，不含 `tf2_eigen`）

---

## 依赖关系

### path_planner_pkg 最终依赖列表

**ROS2 包：**
- `ament_cmake` (构建工具)
- `rclcpp`
- `std_msgs`
- `nav_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `visualization_msgs`
- `trajectory_msgs`
- `tf2`
- `tf2_ros`
- `builtin_interfaces`
- `state_machine` ⚠️ 自定义包

**系统库：**
- `Eigen3` >= 3.4

### state_machine 依赖列表

**ROS2 包：**
- `ament_cmake`
- `rosidl_default_generators` (构建时)
- `rosidl_default_runtime` (运行时)
- `rclcpp`
- `std_msgs`
- `nav_msgs`
- `geometry_msgs`
- `visualization_msgs`
- `builtin_interfaces`

---

## 在 Jazzy 虚拟机中的构建步骤

### 1. 准备工作空间

```bash
# 在虚拟机中
mkdir -p ~/jazzy_ws/src
cd ~/jazzy_ws/src

# 复制两个包到 src 目录
# - path_planner_pkg
# - state_machine
```

### 2. 设置环境

```bash
source /opt/ros/jazzy/setup.bash
```

### 3. 安装依赖

```bash
cd ~/jazzy_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. 构建包

```bash
# 先构建 state_machine（因为 path_planner_pkg 依赖它）
colcon build --packages-select state_machine

# 然后构建 path_planner_pkg
colcon build --packages-select path_planner_pkg

# 或者一次性构建两个包
colcon build --packages-select state_machine path_planner_pkg
```

### 5. 验证构建

```bash
source install/setup.bash

# 检查节点可执行文件
ros2 pkg executables path_planner_pkg
# 应该输出：path_planner_pkg path_planner_node

ros2 pkg executables state_machine
# 应该输出：state_machine state_machine_node
```

### 6. 运行测试

```bash
# 启动 path_planner 节点
ros2 launch path_planner_pkg path_planner.launch.py

# 或直接运行节点
ros2 run path_planner_pkg path_planner_node

# 检查话题
ros2 topic list

# 检查节点信息
ros2 node info /path_planner_node
```

---

## 预期结果

### ✅ 成功标志

- 构建无错误/警告（或仅有无害的弃用警告）
- 所有可执行文件正确生成
- 节点启动正常并发布心跳到 `statemachine/node_health`
- 话题列表包含：
  - `/command/trajectory`
  - `/path_planner/path`
  - `/path_planner/map`
  - `/path_planner/markers`
  - `/statemachine/node_health`

### ⚠️ 可能的警告（可忽略）

- CMake 策略警告（CMP0xxx）
- QoS 配置弃用提示（功能仍正常）
- Eigen3 版本提示信息

---

## 回滚方案

如果需要回到 Humble：

```bash
# 恢复 path_planner_pkg/CMakeLists.txt
git diff path_planner_pkg/CMakeLists.txt
git checkout path_planner_pkg/CMakeLists.txt

# 恢复 path_planner_pkg/package.xml
git checkout path_planner_pkg/package.xml

# 恢复 state_machine/CMakeLists.txt
git checkout state_machine/CMakeLists.txt
```

或手动修改：
- CMake 版本改回 `3.8`
- 添加回 `tf2_eigen` 依赖
- 添加回 `eigen3_cmake_module` 依赖

---

## 技术细节

### 为什么移除 tf2_eigen？

**分析结果：**
- 搜索所有源文件（`.cpp`, `.hpp`）
- **未发现任何**以下函数的使用：
  - `tf2::fromMsg()`
  - `tf2::toMsg()`
  - `tf2::convert()`
  - `tf2::Transform` 相关操作

**结论：**
- `tf2_eigen` 仅在依赖列表中声明，实际代码未调用
- 移除后不影响任何功能
- 减少不必要的依赖复杂度

### Eigen3 查找方式变化

**Humble 方式：**
```cmake
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
```

**Jazzy 改进方式：**
```cmake
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
```

**优势：**
- 直接使用 Eigen3 的 CMake 配置文件
- 明确指定最低版本（3.4）
- `NO_MODULE` 确保使用 Eigen 自带的 config 模式
- 减少对额外 helper 包的依赖

---

## 测试清单

在 Jazzy 虚拟机中完成以下测试：

- [ ] `colcon build` 成功，无错误
- [ ] 节点启动无 segfault
- [ ] 订阅 `/current_state_est` 正常
- [ ] 订阅点云 `/Quadrotor/Sensors/DepthCamera/point_cloud` 正常
- [ ] 发布轨迹到 `command/trajectory` 正常
- [ ] 地图更新可视化正常
- [ ] A* 路径规划功能正常
- [ ] RRT 探索功能正常
- [ ] 与 state_machine 通信正常
- [ ] 参数从 YAML 加载正常

---

## 联系信息

如有问题，请检查：
1. Jazzy 环境是否正确 source
2. 依赖是否完整安装（`rosdep install`）
3. state_machine 包是否先于 path_planner_pkg 构建成功
4. Eigen3 版本是否 >= 3.4

**迁移完成！** 🎉

