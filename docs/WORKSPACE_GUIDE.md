# robot_rl_myself（FishBot ROS 2 工作区）

> 本文保留原房间与 PAL 操作教程；命令已显式选择原房间及 AMCL。当前默认仓库的介绍、切换和测试见 [主 README](../README.md)。

本项目运行于 Ubuntu 22.04 / ROS 2 Humble，包含 FishBot 的模型与 Gazebo
Classic 仿真、Nav2 导航、基础导航示例和自动巡检。当前工作区共有 5 个包。

## 环境与编译

每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/archer/robot_rl/install/setup.bash
source /home/archer/robot_rl_myself/install/setup.bash
```

修改源码后重新编译：

```bash
cd /home/archer/robot_rl_myself
source /opt/ros/humble/setup.bash
source /home/archer/robot_rl/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 五个包的区别与运行命令

### 1. robot_rl_description：机器人模型和仿真底座

提供 FishBot URDF/Xacro、Gazebo 世界、雷达/相机插件、ros2_control
控制器配置。它负责产生机器人、`/scan`、`/odom`、相机话题和 TF，但不提供
路径规划与定位。

```bash
# 只在 RViz 显示模型（不是物理仿真）
ros2 launch robot_rl_description display_robot.launch.py

# 只启动 Gazebo、机器人和控制器（不启动 Nav2）
ros2 launch robot_rl_description gazebo_sim.launch.py

# 无 Gazebo 窗口运行
ros2 launch robot_rl_description gazebo_sim.launch.py gui:=false
```

### 2. robot_rl_navigation2：完整仿真导航入口

提供地图、AMCL、规划器、控制器、行为树及 RViz 配置。默认的一条命令会同时
启动 Gazebo、生成 FishBot、加载控制器、启动 Nav2 和 RViz：

```bash
ros2 launch robot_rl_navigation2 navigation2.launch.py scene:=room localization:=amcl
```

常用参数：

```bash
# 无图形界面，适合服务器测试
ros2 launch robot_rl_navigation2 navigation2.launch.py scene:=room localization:=amcl gui:=false use_rviz:=false

# Gazebo 已在另一个终端运行时，避免重复启动
ros2 launch robot_rl_navigation2 navigation2.launch.py scene:=room localization:=amcl start_gazebo:=false

# 使用其他地图或世界
ros2 launch robot_rl_navigation2 navigation2.launch.py scene:=room localization:=amcl \
  map:=/绝对路径/map.yaml world:=/绝对路径/world.world
```

自定义 Nav2 参数文件时使用 `nav2_params_file:=/绝对路径/nav2_params.yaml`。

默认情况下机器人生成在 Gazebo 世界 `(0, 0, 0)`，AMCL 也会自动使用相同初始
位姿，所以 RViz 可以立即使用 `map` 作为 Fixed Frame。机器人若被改为从其他坐标
生成，需在 RViz 使用 `2D Pose Estimate` 修正定位，再用 `Nav2 Goal` 发送目标。
默认地图和世界必须保持对应，否则定位和规划会失败。

RViz 默认延迟 15 秒启动，给 Gazebo、差速控制器和 AMCL 留出初始化时间，避免
Fixed Frame 为 `map` 时出现启动阶段的临时 TF 报错；可用 `rviz_delay:=秒数` 调整。
启动文件会先检查 Gazebo Classic 的 11345 端口；若旧 `gzserver` 仍在运行，会在
生成机器人和启动 Nav2 前直接退出，避免新 RViz/Nav2 误连旧世界。若确实需要复用
已经运行的 Gazebo，应显式使用 `start_gazebo:=false`。
RViz 顶视图使用 0 度视角，与 Gazebo 顶视图保持 +X 向右、+Y 向上；若手动旋转过
RViz 视角，可在 Views 面板把 TopDownOrtho 的 Angle 恢复为 0。

相机彩色图像话题为 `/camera/image_raw`。运行 `rqt_image_view` 后在左上角
下拉框选择该话题；深度图为 `/camera/depth/image_raw`。Gazebo 相机在没有
订阅者时可能按需停止渲染，打开图像订阅者后会恢复发布，这是正常的性能优化。

### 3. robot_rl_application：基础导航 API 示例

这是调用 Nav2 的 Python 示例，不负责启动 Gazebo 或 Nav2。必须先运行
`robot_rl_navigation2`，再在另一个已 source 的终端运行：

```bash
ros2 run robot_rl_application init_robot_pose    # 初始位姿固定为 (0, 0, 0)
ros2 run robot_rl_application get_robot_pose     # 持续显示 map -> base_link
ros2 run robot_rl_application nav_to_pose        # 固定导航至 (2, 1)
ros2 run robot_rl_application waypoint_follower  # 固定执行三个路点
```

这些程序当前是教学示例，坐标写在 Python 源码中，没有命令行坐标参数。

### 4. autopatrol_interfaces：自动巡检的消息接口

只定义 `SpeechText.srv`（请求文本、返回是否成功），自身不会启动节点：

```bash
ros2 interface show autopatrol_interfaces/srv/SpeechText
```

### 5. autopatrol_robot：自动巡检业务

调用 Nav2 按配置文件循环巡检；到点后读取深度相机图像并保存到
`/tmp/autopatrol_images`，同时通过 `SpeechText` 服务语音播报。必须先启动完整导航：

```bash
# 终端 1
ros2 launch robot_rl_navigation2 navigation2.launch.py scene:=room localization:=amcl

# 终端 2
ros2 launch autopatrol_robot autopatrol.launch.py scene:=room
```

也可单独运行：

```bash
ros2 run autopatrol_robot speaker
ros2 run autopatrol_robot patrol_node --ros-args \
  --params-file /home/archer/robot_rl_myself/install/autopatrol_robot/share/autopatrol_robot/config/patrol_config.yaml
```

## 推荐使用流程

最简单的人工导航只需运行：

```bash
ros2 launch robot_rl_navigation2 navigation2.launch.py scene:=room localization:=amcl
```

Gazebo 出现机器人且 RViz 的 Nav2 节点激活后，可直接在 RViz 设置目标点。
只有机器人实际生成位置不是默认 `(0, 0, 0)` 时才需要重新设置初始位姿。
自动巡检则保持该命令运行，再启动 `autopatrol.launch.py`。

## 使用迁移的 PAL Gazebo 世界

`pal_gazebo_worlds` 的 69 个 `.world/.sdf` 已迁移到：

```text
robot_rl_description/world/pal/
```

它们引用的 PAL 模型与网格也已迁移到 `robot_rl_description/models/pal/` 和
`robot_rl_description/meshes/pal/`，launch 会自动设置 Gazebo 模型搜索路径。
列出可选场景：

```bash
find /home/archer/robot_rl_myself/install/robot_rl_description/share/robot_rl_description/world/pal \
  -maxdepth 1 -type f | sort
```

只测试机器人在某个世界中的模型、控制器、雷达和相机（不启动导航）：

```bash
PAL_WORLD=/home/archer/robot_rl_myself/install/robot_rl_description/share/robot_rl_description/world/pal
ros2 launch robot_rl_description gazebo_sim.launch.py \
  world:=$PAL_WORLD/empty_room.world
```

如果 `(0, 0)` 有障碍物，通过生成坐标参数选择空闲位置：

```bash
ros2 launch robot_rl_description gazebo_sim.launch.py \
  world:=$PAL_WORLD/pal_office.world \
  spawn_x:=1.0 spawn_y:=1.0 spawn_yaw:=1.57
```

### 在新世界中 SLAM 建图

PAL 世界不附带占据栅格地图。首次使用布局不同的世界时，以 SLAM 模式启动：

```bash
PAL_WORLD=/home/archer/robot_rl_myself/install/robot_rl_description/share/robot_rl_description/world/pal
ros2 launch robot_rl_navigation2 navigation2.launch.py scene:=room localization:=amcl \
  world:=$PAL_WORLD/empty_room.world slam:=true
```

在 RViz 使用 Nav2/手动速度控制机器人遍历场景后，另开终端保存地图：

```bash
source /opt/ros/humble/setup.bash
source /home/archer/robot_rl_myself/install/setup.bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/archer/robot_rl_myself/src/robot_rl_navigation2/maps/pal_empty_room
```

这会生成 `pal_empty_room.yaml` 和 `pal_empty_room.pgm`。结束 SLAM，重新编译使地图
安装到 share 目录：

```bash
cd /home/archer/robot_rl_myself
colcon build --symlink-install --packages-select robot_rl_navigation2
source install/setup.bash
```

### 使用保存的地图导航

世界文件和地图必须成对选择：

```bash
PAL_WORLD=/home/archer/robot_rl_myself/install/robot_rl_description/share/robot_rl_description/world/pal
NAV_MAP=/home/archer/robot_rl_myself/install/robot_rl_navigation2/share/robot_rl_navigation2/maps
ros2 launch robot_rl_navigation2 navigation2.launch.py scene:=room localization:=amcl \
  world:=$PAL_WORLD/empty_room.world \
  map:=$NAV_MAP/pal_empty_room.yaml
```

若机器人生成点不是地图中的 `(0,0)`，同时传入 `spawn_x/spawn_y/spawn_yaw`，并在
RViz 用 `2D Pose Estimate` 给 AMCL 设置相同位置。

注意：这些上游场景最初为 PAL 机器人测试制作，不保证每个场景都适合 FishBot。
`cabinet_grasping.world` 和 `calibration_room.world` 含有原开发机的绝对 mesh 路径；
另有少数场景引用未随上游包提供的模型（如 `reemc`、`stairs_talos`）。这些场景可能
缺少部分视觉模型，优先使用 `empty_room.world`、`pal_office.world`、
`small_office.world`、`hospital.world`、`home.world` 等自包含场景进行测试。

## 完整示例：为 PAL home.world 建图并导航

下面以迁移后的 `home.world` 为例。建议第一次先用 `gui:=true`，确认机器人没有
生成在家具或墙体内部。以下每个“终端”都可以直接打开；ROS 2 环境已由
`~/.bashrc` 自动加载。

### 0. 确认环境与清理旧进程

新开终端，确认当前 overlay 被识别：

```bash
ros2 pkg prefix robot_rl_description
ros2 pkg prefix robot_rl_navigation2
```

两条命令都应指向 `/home/archer/robot_rl_myself/install/...`。开始前关闭其他正在
运行的 Gazebo、Nav2 和 RViz 窗口，避免重复的 `/controller_manager` 和节点名称。

### 1. 启动 home.world 的 SLAM 建图模式

终端 1：

```bash
PAL_WORLD=/home/archer/robot_rl_myself/install/robot_rl_description/share/robot_rl_description/world/pal

ros2 launch robot_rl_navigation2 navigation2.launch.py scene:=room localization:=amcl \
  world:=$PAL_WORLD/home.world \
  slam:=true \
  spawn_x:=0.0 spawn_y:=0.0 spawn_z:=0.15 spawn_yaw:=0.0
```

等待 Gazebo 出现 FishBot、控制器加载完成以及 RViz 打开。SLAM 模式中 RViz 的
Fixed Frame 应为 `map`。如果 `(0,0)` 恰好在障碍物内，停止 launch，在 Gazebo 中
找到空闲位置，然后修改 `spawn_x`、`spawn_y` 和 `spawn_yaw` 后重新启动。

检查关键数据是否正常：

```bash
ros2 topic hz /scan
ros2 topic echo /map --once --field info
```

`/scan` 应持续有频率，`/map` 应能输出地图宽度、高度和分辨率。

### 2. 用键盘控制机器人遍历房间

终端 2：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

常用按键：

- `i`：前进；`,`：后退。
- `j`/`l`：原地左转/右转。
- `u`/`o`：左前/右前。
- `k`：立即停止。
- `q`/`z`：提高/降低全部速度。

建图时缓慢沿墙行驶，让激光看到房间边界和家具。经过门口、走廊拐角和大房间时
多转一圈，使扫描产生重叠。RViz 中灰色未知区域应逐渐变成白色空闲区域和黑色
障碍物。避免高速运动、撞墙或长期只沿一条直线行驶，否则 SLAM 容易漂移。

### 3. 检查并保存地图

当主要可通行区域都已覆盖后，让机器人停止。终端 3：

```bash
mkdir -p /home/archer/robot_rl_myself/src/robot_rl_navigation2/maps

ros2 run nav2_map_server map_saver_cli \
  -f /home/archer/robot_rl_myself/src/robot_rl_navigation2/maps/pal_home
```

看到 `Map saved` 后检查：

```bash
ls -lh /home/archer/robot_rl_myself/src/robot_rl_navigation2/maps/pal_home.*
```

应有两个文件：

```text
pal_home.pgm
pal_home.yaml
```

不要手工猜测 `pal_home.yaml` 中的 `origin`；它由 SLAM 保存，表示图像左下角在
地图坐标系的位置。world 和地图是一对，移动墙壁或大型固定家具后应重新建图。

### 4. 停止 SLAM并安装新地图

在终端 1 按 `Ctrl+C`，等待 Gazebo、Nav2 和 RViz 完全退出。然后编译导航包：

```bash
cd /home/archer/robot_rl_myself
colcon build --symlink-install --packages-select robot_rl_navigation2
source install/setup.bash
```

### 5. 使用 pal_home 地图进行 AMCL 导航

```bash
PAL_WORLD=/home/archer/robot_rl_myself/install/robot_rl_description/share/robot_rl_description/world/pal
NAV_MAP=/home/archer/robot_rl_myself/install/robot_rl_navigation2/share/robot_rl_navigation2/maps

ros2 launch robot_rl_navigation2 navigation2.launch.py scene:=room localization:=amcl \
  world:=$PAL_WORLD/home.world \
  map:=$NAV_MAP/pal_home.yaml \
  spawn_x:=0.0 spawn_y:=0.0 spawn_z:=0.15 spawn_yaw:=0.0
```

注意：SLAM 建图时地图坐标系的原点和 Gazebo 世界原点不一定相同。导航启动后，
在 RViz 用 `2D Pose Estimate` 点击机器人在 `pal_home` 地图上的真实位置并拖动设置
朝向；看到激光点与地图墙壁重合后，再用 `Nav2 Goal` 下发目标。更换生成坐标后也
必须重新设置初始位姿。

### 6. home.world 常见问题

- Gazebo 有场景但没有机器人：检查终端中 `SpawnEntity` 和控制器错误，并确认生成
  坐标不在墙体或家具内部。
- RViz 地图不增长：检查 `/scan`、`/tf` 和 `/clock`，并确认启动参数确实为
  `slam:=true`。
- 机器人不响应键盘：确保键盘终端保持焦点，检查
  `ros2 topic hz /cmd_vel` 和 `ros2 control list_controllers`。
- 保存后导航地图错位：不要复用 `room.yaml`；必须选择 `pal_home.yaml`，并重新用
  `2D Pose Estimate` 对齐初始位姿。
- 部分小物件没有纹理：上游 `home.world` 引用了少数未随 PAL 包提供的模型资源，
  不影响墙壁、激光建图、机器人控制和 Nav2，Gazebo 可能只显示相关资源警告。

## 功能检查与限制

- 5 个包均可被 ROS 2 索引，接口和 6 个 Python 可执行程序均已安装。
- Xacro、Gazebo/ros2_control、Nav2 和 RViz 依赖已安装。
- `navigation2.launch.py` 已支持一条命令启动 Gazebo 与导航；也可关闭其中任一 GUI。
- `robot_rl_application` 的目标点是源码中的固定值，不是通用命令行工具。
- 自动巡检依赖系统音频设备和 `espeak-ng`；无声卡环境不影响导航，但语音可能
  无法播放。图像只有在 `/camera/image_raw` 收到数据后才会保存。
- `patrol_config.yaml` 的所有巡检点必须位于地图空闲区域，否则相应目标会失败。
- 更换 Gazebo 世界后，若仍需基于地图的 AMCL 导航，必须准备与新世界对应的
  占据栅格地图；可先使用 SLAM 建图并保存，再切回 AMCL 导航。仅更改纹理、灯光
  或不影响二维障碍物的物件时，可以继续沿用原地图。
- Gazebo Classic GUI 需要可用的 `DISPLAY` 和 OpenGL；远程或无桌面环境请使用
  `gui:=false use_rviz:=false`。

常用诊断命令：

```bash
ros2 pkg prefix robot_rl_navigation2
ros2 topic list | grep -E 'scan|odom|camera|cmd_vel'
ros2 control list_controllers
ros2 lifecycle nodes
```
