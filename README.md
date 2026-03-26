# 基于多传感器融合的计算机教室巡检物联网机器人

## 系统概述

基于 MentorPi 机器人平台（树莓派5 + 麦克纳姆轮底盘 + 激光雷达 + 深度相机）的计算机教室自主巡检系统。

### 核心功能
- SLAM 建图与定位（激光雷达为主，深度相机辅助）
- 自主巡航与路径规划（Nav2 + 弓字形覆盖）
- 电源指示灯状态识别（HSV 颜色分析 + 轮廓检测）
- 巡检数据记录（CSV/JSON/SQLite）

### 系统架构
```
┌─────────────────────────────────────────────┐
│              控制调度层 (Control)             │
│  InspectionManager  │  Scheduler            │
├─────────────────────────────────────────────┤
│   定位导航层 (Nav)   │   视觉识别层 (Vision)  │
│  SLAM  PathPlanner  │  PowerLedDetector     │
│  MotionController   │  ImageProcessor       │
├─────────────────────────────────────────────┤
│              传感器层 (Sensor)                │
│  LidarInterface  │  CameraInterface        │
│              SensorManager                  │
├─────────────────────────────────────────────┤
│              数据层 (Data)                    │
│              DataRecorder                   │
└─────────────────────────────────────────────┘
```

## 环境要求

- MentorPi 标准版（麦克纳姆轮底盘）
- 树莓派 5
- ROS2 Humble
- Python 3.10+
- OpenCV 4.x

## 安装

```bash
# 1. 将项目放入 ROS2 工作空间
mkdir -p ~/inspection_ws/src
cp -r mentor_pi_inspection ~/inspection_ws/src/

# 2. 安装依赖
cd ~/inspection_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. 编译
colcon build --packages-select mentor_pi_inspection
source install/setup.bash
```

## 使用流程

### 第一步：建图

在教室环境中手动遥控机器人走一圈，建立地图：

```bash
# 终端 1：启动 SLAM 建图
ros2 launch mentor_pi_inspection slam_launch.py

# 终端 2：启动键盘遥控
ros2 run mentor_pi_inspection keyboard_teleop

# 键位：
#   w/s 前进/后退
#   a/d 左右平移（麦克纳姆底盘）
#   q/e 左右旋转
#   space 或 k 急停

# 建图完成后，保存地图：
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/inspection_maps/classroom_map'}"

# 同时保存 nav2 格式地图
ros2 run nav2_map_server map_saver_cli -f ~/inspection_maps/classroom_map
```

### 第二步：设置巡检路径

```bash
# 方法1：手动添加巡检点
ros2 topic pub /inspection/command std_msgs/msg/String \
  "{data: '{\"action\": \"add_waypoint\", \"x\": 1.0, \"y\": 0.5, \"yaw\": 0.0}'}"

# 方法2：自动生成网格覆盖路径
ros2 topic pub /inspection/command std_msgs/msg/String \
  "{data: '{\"action\": \"generate_grid\", \"x_min\": -3, \"x_max\": 3, \"y_min\": -2, \"y_max\": 2, \"spacing\": 1.5}'}"

# 保存路径点
ros2 topic pub /inspection/command std_msgs/msg/String \
  "{data: '{\"action\": \"save_waypoints\"}'}"
```

### 第三步：启动巡检

```bash
# 完整系统启动（导航 + 视觉 + 记录）
ros2 launch mentor_pi_inspection bringup_launch.py

# 发送开始巡检命令
ros2 topic pub --once /inspection/user_command std_msgs/msg/String "{data: 'start'}"

# 查看巡检状态
ros2 topic echo /inspection/state

# 查看电源检测结果
ros2 topic echo /inspection/power_status

# 停止巡检
ros2 topic pub --once /inspection/user_command std_msgs/msg/String "{data: 'stop'}"
```

### 查看巡检数据

巡检数据默认保存在 `~/inspection_data/` 目录下，格式为 CSV。

```bash
# 查看最新巡检记录
cat ~/inspection_data/*.csv
```

## 调试工具

```bash
# 查看调试图像（带 LED 检测标注）
ros2 run rqt_image_view rqt_image_view /inspection/debug_image

# 查看传感器状态
ros2 topic echo /inspection/sensor_status

# 在 RViz2 中查看地图和导航
rviz2
```

## 可移植性说明

- 所有传感器 topic 通过参数配置，不写死设备路径
- 配置文件集中在 `config/` 目录
- 相同型号机器人之间可直接部署，无需修改代码
- 只需根据实际环境重新建图和设置巡检路径

## 附录：SLAM 建图键盘控制与操作步骤

本节是对“第一步：建图”的补充说明，不修改上面的原始流程，只补充实际操作细节。

### 1. 建图前需要先做什么

```bash
# 在工作空间重新编译一次（新增了 keyboard_teleop 可执行节点）
cd ~/inspection_ws
colcon build --packages-select mentor_pi_inspection

# 每个新终端都要 source
source ~/inspection_ws/install/setup.bash
```

### 2. 建图时你要按什么顺序操作

```bash
# 终端 1：启动 SLAM
ros2 launch mentor_pi_inspection slam_launch.py

# 终端 2：启动键盘控制
ros2 run mentor_pi_inspection keyboard_teleop

# 可选：终端 3 查看地图
rviz2
```

实际建图建议按下面顺序走：

1. 先让机器人原地小角度旋转，确认 `/map` 开始更新。
2. 贴着教室外沿慢速走一圈，优先把墙边、角落、门口轮廓扫完整。
3. 再走中间通道，补齐桌椅之间的区域，保证有一定重叠扫描。
4. 转弯和狭窄位置要慢，不要急停急转，避免雷达匹配发散。
5. 地图闭环后回到起点附近，再保存地图。

### 3. 键盘控制映射

- `w`：前进
- `s`：后退
- `a`：左平移
- `d`：右平移
- `q`：左转
- `e`：右转
- `space` 或 `k`：急停
- `Ctrl-C`：退出键盘控制

说明：`slam_launch.py` 现在会同时启动 `motion_controller`，键盘控制发布到 `/inspection/cmd_vel`，再由控制器转发到 `/cmd_vel`。

### 4. 建图完成后要做什么

```bash
# 保存 slam_toolbox 图
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/inspection_maps/classroom_map'}"

# 保存 nav2 地图
ros2 run nav2_map_server map_saver_cli -f ~/inspection_maps/classroom_map
```

确认以下文件存在即可：

- `~/inspection_maps/classroom_map.data`
- `~/inspection_maps/classroom_map.posegraph`
- `~/inspection_maps/classroom_map.pgm`
- `~/inspection_maps/classroom_map.yaml`
