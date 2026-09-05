# robot_rl_navigation2 导航算法与定位说明

## 启动

```bash
ros2 launch robot_rl_navigation2 navigation2.launch.py
```

新终端已通过 `~/.bashrc` 自动加载 ROS 2 和工作空间。不要重复执行这条 launch；
启动文件会检测 Gazebo 11345 端口，防止第二套 Nav2/RViz 误连旧仿真。

## 本次问题的真实原因

截图中机器人停在右侧墙边，激光轮廓与地图相差很大。现场数据为：

- wheel odom：`(9.85, -1.67)`；
- AMCL：`(6.19, -3.74)`；
- 机器人已经在右侧墙附近发生碰撞和车轮打滑。

旧配置用轮子编码器积分 `/odom`。机器人被墙挡住时轮子仍能旋转，odom 继续增长，
AMCL 无法用激光一次修正数米误差，最终 `map→odom→base_link` 错误。此时 RViz 中
激光、局部代价地图、路径都会偏离真实墙体，progress checker 会报告无进展；这不是
单独更换全局规划算法就能解决的问题。

旧局部控制器 DWB 还存在以下不适合当前机器人的设置：

- `required_movement_radius=0.5 m`，对直径约 0.34 m 的小机器人过大；
- 到点容差仅 0.05 m，在带噪定位下容易在终点反复调整；
- 只将 2.5 m 内激光加入代价地图，无法充分利用 8 m 雷达；
- DWB 依靠离散速度采样和 critic 权重，靠墙或急转弯时容易振荡。

## 当前定位与 TF 架构

本项目默认是 Gazebo 仿真，并且 `custom_room.world` 与 `room.yaml` 使用相同米制坐标。
因此导航采用仿真 ground-truth 定位：

```text
map --ground_truth_odom_tf(identity)--> odom
odom --Gazebo P3D--> base_footprint --URDF--> base_link --URDF--> laser_link
```

- `/odom`：Gazebo `libgazebo_ros_p3d.so`，反映实体真实碰撞、侧滑和受阻状态；
- `/wheel_odom`：差速控制器轮速积分，仅供诊断，不发布 TF；
- `map→odom`：`ground_truth_odom_tf` 唯一发布，仿真中为 identity；
- `odom→base_footprint`：`ground_truth_odom_tf` 根据 `/odom` 发布；
- AMCL 仍订阅地图和激光、可显示粒子，但 `tf_broadcast=false`，不会与仿真真值争抢 TF。

这个设计只用于 Gazebo 仿真。迁移到真实机器人时必须移除 P3D/identity TF，恢复
AMCL 的 `tf_broadcast=true`，并使用真实编码器、IMU 或 robot_localization 输出 odom。

## 算法升级

| 模块 | 旧配置 | 当前配置 | 改进 |
|---|---|---|---|
| 全局规划 | NavFn，Dijkstra | SmacPlanner2D | 代价感知 A*，路径更平滑、贴障更少 |
| 局部控制 | DWB | Regulated Pure Pursuit | 曲率/障碍降速、前向碰撞预测 |
| 速度平滑 | OPEN_LOOP | CLOSED_LOOP | 根据真实 `/odom` 平滑速度 |
| 局部障碍 | VoxelLayer | ObstacleLayer | 与单层 2D LaserScan 语义一致 |
| odom | wheel odom | Gazebo ground truth | 碰撞打滑时不再虚假前进 |

RPP 主要安全参数：

- 期望线速度 0.22 m/s；
- lookahead 0.20–0.60 m；
- 碰撞预测时间 1.2 s；
- 靠近障碍、急弯和终点时最低降至 0.05 m/s；
- 原地转向阈值 45°，禁止倒车；
- 机器人半径 0.17 m，膨胀半径 0.35 m。

SmacPlanner2D 使用 0.05 m 原始代价地图、不降采样，规划时间上限 2 s，并对高代价
区域施加更高旅行代价，减少贴墙路径。

## 激光与代价地图

激光统一为标准 ROS 角度：

```text
angle_min = -pi
angle_max = +pi
range = 0.12 ... 8.0 m
frame_id = laser_link
```

局部和全局 ObstacleLayer 使用 `/scan`，清障距离 8 m、障碍标记距离 7.5 m。
RViz 中 LaserScan 应贴合地图墙体。若不贴合，依次检查：

```bash
ros2 topic info /odom -v       # 应只有 ground_truth_odometry 发布
ros2 topic info /wheel_odom -v # 只供诊断
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 topic echo /scan --once
```

不要用 RViz 的 `2D Pose Estimate` 修正默认仿真真值模式；它只会改变 AMCL 粒子，
不会改变 ground-truth TF。如果换成真实机器人模式，才由 AMCL 初始位姿负责定位。

## 目标点与“停住”的判断

目标点必须在白色自由栅格内，并离黑色墙体/粉色膨胀区至少一个机器人半径。若目标
落在墙内或膨胀区，规划器应拒绝它，这是正确安全行为。

诊断命令：

```bash
ros2 action info /navigate_to_pose
ros2 topic echo /cmd_vel
ros2 topic echo /odom --once
ros2 topic echo /wheel_odom --once
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
```

虚拟机中 Gazebo 实时因子可能低于 1，现实时间会比仿真时间长。应查看 `/cmd_vel`、
`/odom` 和 action 结果判断是否真的停住，不要仅按墙钟等待时间判断。

## 更换世界与地图

```bash
ros2 launch robot_rl_navigation2 navigation2.launch.py \
  world:=/绝对路径/example.world \
  map:=/绝对路径/example.yaml
```

ground-truth identity 定位要求新地图坐标原点与 Gazebo world 坐标一致。如果地图由
SLAM 建立且 origin/朝向不同，应切回 AMCL 定位或正确计算地图 origin；不能同时让
AMCL 和 ground_truth 节点发布 `map→odom`。

## 修改文件和备份

主要文件：

- `config/nav2_params.yaml`
- `../robot_rl_description/urdf/fishbot/fishbot.ros2_control.xacro`
- `../robot_rl_description/urdf/fishbot/plugins/gazebo_sensor_plugin.xacro`
- `../robot_rl_description/scripts/ground_truth_odom_tf.py`

升级前备份位于工作空间：

```text
.backups/nav2_algorithm_upgrade_20260716/
```

## 实际验收结果（2026-07-16）

- `planner_server` active，实际加载 `nav2_smac_planner/SmacPlanner2D`；
- `controller_server` active，实际加载
  `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`；
- `/odom` Publisher count=1，发布者为 `ground_truth_odometry`；
- `/wheel_odom` Publisher count=1，不发布 odom TF；
- AMCL `tf_broadcast=false`；
- `map→odom` 为 identity，发布者只有 `ground_truth_odom_tf`；
- 激光为 `[-3.14159, 3.14159]`、0.12–8.0 m；
- 2.0 m 目标和最终 0.5 m 目标均返回 `SUCCEEDED`；
- 最终目标后 Gazebo、`/odom` 与 `map→base_footprint` 均约为
  `(0.383, -0.011)`，坐标一致；同期 `/wheel_odom` 约为
  `(0.382, -0.014)`，仅作为轮滑诊断数据。
