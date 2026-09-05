# Nav2 地图、TF 与摄像头诊断修复报告

日期：2026-07-16  
工作空间：`/home/archer/robot_rl_myself`  
环境：ROS 2 Humble、Gazebo Classic 11.10.2、`rmw_fastrtps_cpp`

## 1. 原始现象

- 曾出现 `/map` 无发布者、RViz Fixed Frame=`map` 报错及 `No map received`。
- RViz 地图被认为与 Gazebo `custom_room.world` 不一致。
- 摄像头原话题为非规范的 `/camera_sensor/image_raw`，rqt 中不便查找；仿真启动早期还存在传感器/TF 尚未就绪便启动 Nav2 的竞争。
- 更早的 launch 曾把小写 `false` 放入 Python 表达式，产生 `name 'false' is not defined`。

## 2. 真实根因

### `/map` 与 RViz

当前源码的静态地图链路本身能够工作：`map_server` 由 Nav2 localization lifecycle manager 管理，能够进入 `active` 并发布 `/map`。原先“Publisher count: 0”不是地图文件损坏，而是旧版本 launch/未重新编译安装或启动初始化阶段的观测。主要问题是 Gazebo、控制器、传感器、Nav2 和 RViz 同时启动，RViz/AMCL 在 TF 与 `/scan` 就绪前开始工作，出现启动阶段的 No map/TF filter 错误；另外原 launch 的小写布尔值表达式会直接中止启动。

修复方式：保留已完成的布尔规范化，并让 Nav2 延迟 8 秒、RViz 延迟 15 秒启动；RViz 改用项目内受版本控制的配置。

### 地图与 Gazebo 对齐

默认地图确实对应默认 `custom_room.world`。地图参数为 377×222、0.05 m/px，覆盖范围为 x=[-10.40, 8.45]、y=[-6.49, 4.61]，与 world 外墙约 x=[-10.3837, 8.466]、y=[-6.494, 4.606] 一致。地图 `origin: [-10.4, -6.49, 0]` 正确，不应修改。机器人和 AMCL 初始位姿均为 `(0,0,0)`，也不应改 spawn pose。

因此没有添加静态 `map→odom`，也没有随机调整 origin。若用户选择了其他 world 却仍加载 `room.yaml`，地图当然不会一致；必须将对应 world 与对应 map YAML 成对传入。

### 摄像头与 rqt_image_view

Gazebo Classic 相机插件工作正常，但原 `<sensor name="camera_sensor">` 导致公开话题为 `/camera_sensor/image_raw`，而应用、文档和用户预期不统一。插件现在显式设置根 namespace 和 `camera_name=camera`，规范话题为 `/camera/image_raw` 与 `/camera/camera_info`。分辨率由 800×600 调为 640×480，降低虚拟机软件渲染负载；补全 near/far clip。自动巡检订阅同步更新。

实际启动 `rqt_image_view /camera/image_raw` 后，ROS 图显示 rqt 节点已订阅该话题，且持续收到图像。

## 3. 当前运行模式

默认采用静态地图定位导航：`map_server + AMCL + Nav2`，不是 SLAM。`map→odom` 只由 AMCL 发布。需要为新世界建图时可用 `slam:=true` 启动 SLAM 模式，保存地图后再切回默认 AMCL 模式；不能同时让 AMCL 和 SLAM 发布同一变换。

## 4. 修改文件

1. `src/robot_rl_navigation2/launch/navigation2.launch.py`
   - 使用项目 RViz 配置 `config/fishbot_nav2.rviz`。
   - 新增 `nav2_delay`（默认 8 秒），延迟 Nav2 到 Gazebo、TF 和传感器就绪后启动。
   - 保留 `rviz_delay`（默认 15 秒）及正确的 `slam` 布尔规范化。
2. `src/robot_rl_navigation2/config/fishbot_nav2.rviz`
   - 固定坐标系为 `map`，Map 为 `/map`（Transient Local/Reliable），LaserScan 为 `/scan`（Best Effort），启用 RobotModel。
3. `src/robot_rl_description/urdf/fishbot/plugins/gazebo_sensor_plugin.xacro`
   - Gazebo Classic `libgazebo_ros_camera.so` 显式使用 `/` namespace 和 `camera` 名称。
   - 图像改为 640×480，clip 为 0.05–20 m，frame 为 `camera_optical_link`。
4. `src/autopatrol_robot/autopatrol_robot/patrol_node.py`
   - 图像订阅改为 `/camera/image_raw`。
5. `README.md`
   - 更新摄像头话题、启动与故障排查说明。

备份目录：`.backups/nav2_map_tf_camera_before_fix_20260716/`。该目录保存了本轮能够取得的 README 和巡检节点修改前副本；其余文件在本轮接手时已经处于修改状态，因此不能伪造“修改前”备份。

## 5. 地图 YAML 前后差异

无差异。`src/robot_rl_navigation2/maps/room.yaml` 保持：

```yaml
image: room.pgm
resolution: 0.05
origin: [-10.4, -6.49, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

原因是其比例、旋转和边界均与默认 world 匹配。修改它反而会制造错位。

## 6. TF 树前后差异与最终发布者

TF 拓扑没有被错误改写，只改善启动顺序：

```text
map --AMCL--> odom --fishbot_diff_drive_controller--> base_footprint/base_link
                                                      ├── laser_link
                                                      └── camera_link --robot_state_publisher--> camera_optical_link
```

- `map→odom`：AMCL。
- `odom→base_link` 链：`fishbot_diff_drive_controller`。
- `base_link→laser_link`、`base_link→camera_link→camera_optical_link`：`robot_state_publisher`（静态关节）。
- 未新增任何静态 `map→odom`，不存在第二个 map 变换发布者。
- `camera_link→camera_optical_link` 使用正确光学坐标旋转，图像 `header.frame_id=camera_optical_link`。

Fast DDS 新命令行进程最初约 1–2 秒可能打印 frame does not exist；发现完成后所有 `tf2_echo` 均持续输出，这不是 TF 断链。

## 7. 摄像头前后差异

| 项目 | 修改前 | 修改后 |
|---|---|---|
| 彩色图像 | `/camera_sensor/image_raw` | `/camera/image_raw` |
| CameraInfo | `/camera_sensor/camera_info` | `/camera/camera_info` |
| 分辨率 | 800×600 | 640×480 |
| 编码 | rgb8 | rgb8 |
| frame | camera_optical_link | camera_optical_link |
| 实测图像率 | 约 5–9 Hz（负载相关） | 5.46 Hz（本次虚拟机验收） |

插件配置目标更新率为 10 Hz。实测低于 10 Hz 是当前虚拟机 Gazebo Classic 软件渲染/CPU 负载造成；消息持续、尺寸正确，rqt QoS（Best Effort）与发布者兼容。

## 8. Lifecycle 与传感器验收

- `/map_server`: `active [3]`
- `/amcl`: `active [3]`
- localization manager: `node_names=[map_server, amcl]`, autostart=true
- 所有 6 个 ros2_control 控制器 active
- `/map`: 1 个发布者（map_server），377×222，0.05 m/px，Transient Local/Reliable
- `/scan`: 持续发布，实测约 9.35 Hz，frame=`laser_link`
- `/odom`: 持续发布
- `/camera/image_raw`: 1 个发布者，640×480 rgb8，frame=`camera_optical_link`
- `/camera/camera_info`: 有效，640×480，同一 frame
- rqt 启动后 `/camera/image_raw` subscription count=1，订阅者为 `rqt_gui_cpp_node_*`

## 9. 编译

```bash
cd /home/archer/robot_rl_myself
source /opt/ros/humble/setup.bash
source /home/archer/robot_rl/install/setup.bash
colcon build --symlink-install --packages-select \
  robot_rl_description robot_rl_navigation2 autopatrol_robot \
  --event-handlers console_direct+
source install/setup.bash
```

结果：`Summary: 3 packages finished [2.33s]`，无编译失败。

## 10. 启动命令

`.bashrc` 已自动 source ROS、上游工作空间及本工作空间，新终端可直接执行：

```bash
cd /home/archer/robot_rl_myself
ros2 launch robot_rl_navigation2 navigation2.launch.py
```

图像：

```bash
rqt_image_view /camera/image_raw
```

指定成对的 world/map：

```bash
ros2 launch robot_rl_navigation2 navigation2.launch.py \
  world:=/绝对路径/example.world map:=/绝对路径/example.yaml
```

## 11. 导航运动验收

发送目标 `(map: x=0.5, y=0.0, yaw=0)`：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

结果：Goal accepted；机器人实际位置由 x=0.000 移至约 x=0.451 m；约 7.9 秒完成；`Goal finished with status: SUCCEEDED`，recoveries=0。因此全局规划、局部控制、`cmd_vel`、控制器及 Gazebo 运动链均通过。

## 12. RViz 验收

项目 RViz 配置的 Fixed Frame=`map`、Map Topic=`/map`。运行时 `/map` 有有效发布者，AMCL 提供连续 `map→odom`，`odom→base_link` 和传感器 TF 完整，故不再存在真实的 Fixed Frame/No map 条件。远程自动验收采用 `use_rviz:=false gui:=false` 避免占用桌面，但上述 RViz 所需的底层数据与配置均已逐项验证。

## 13. 新世界是否需要建图

若新 world 的二维墙体/障碍布局改变，AMCL 导航必须先获得与它对应的地图：用 `slam:=true` 建图，使用 `map_saver_cli` 保存，再用保存的 YAML 以默认静态地图模式启动。若只改灯光、纹理或不影响激光二维轮廓的装饰物，可继续使用原地图。world 和 map 必须成对选择。

## 14. 尚存风险

- 虚拟机图像帧率受 CPU/GPU/OpenGL 能力影响，本次约 5.46 Hz，低于插件目标 10 Hz，但连续可用。
- GUI 最终显示效果依赖用户桌面的 DISPLAY/OpenGL；自动测试确认 rqt 进程及订阅链路，无法替代人工观察窗口颜色是否符合偏好。
- 更换 world 后若沿用 `room.yaml`，必然错位；这属于配置组合错误，而非 TF 可自动修正的问题。
- 工作空间不是 Git 仓库，无法生成真实 `git diff` 或提交记录。

## 15. 变更摘要（代替 git diff）

- launch：增加 Nav2 延迟；项目 RViz 配置；保留合法 Python 布尔表达式。
- RViz：固定 frame/map/scan 配置并启用机器人模型。
- Xacro：规范摄像头 namespace/topic，降低分辨率并补全 clip。
- patrol：订阅新图像话题。
- README：同步运行命令和限制说明。
- 地图 YAML、Nav2 TF frame 参数：未修改；spawn 的 x/y/yaw 未修改，第二轮将 z
  从 0.15 m 修正为轮子接地高度 0.002 m，详见下一节。

## 16. 2026-07-16 第二轮初始位姿与重复 Gazebo 修复

用户复测日志中的 `Unable to start server[bind: Address already in use]` 和
`Entity [fishbot] already exists` 由两套 launch 同时运行导致。旧 `gzserver` PID
607801 占用 11345，第二套 RViz/Nav2 连接旧服务器，因此地图、TF、激光和旧机器人
状态被跨实例混用。

进一步冷启动诊断还发现，机器人原先从 `z=0.15 m` 落地，机械臂位置控制器收到首条
轨迹前会受重力下垂并带动车轮；同时 diff-drive 使用 `open_loop=true`，Gazebo 实体
已经移动而 odom 仍显示零。无 `/cmd_vel` 时曾实测约 7 mm/s 漂移。

第二轮修改：

- `navigation2.launch.py`：启动前检查 `GAZEBO_MASTER_URI`（默认 127.0.0.1:11345）。
  端口已占用时，在启动机器人、Nav2 和 RViz 前退出，并提示关闭旧进程；明确使用
  `start_gazebo:=false` 时允许复用已有 Gazebo。
- `navigation2.launch.py`、`gazebo_sim.launch.py`：默认 `spawn_z` 从 0.15 改为
  0.002 m，使车轮刚好接地，避免落地冲量。
- `fishbot_ros2_controller.yaml`：`open_loop=false`，odom 使用轮子实际状态。
- `manipulator.urdf.xacro`：仅对位置控制的可动臂段和夹爪关闭 Gazebo 重力，避免
  控制器初始化前自由下坠；底盘、车轮、碰撞和导航物理保持启用。
- `fishbot_nav2.rviz`：TopDownOrtho Angle 从 -1.5708 改为 0。原值把地图画面旋转
  90°，造成 RViz 竖向、Gazebo 顶视图横向的视觉错觉。

最终静止验收连续 14 秒：Gazebo 位姿仅从 `(0.000073,-0.000016)` 变化到
`(0.000102,-0.000021)` m，约 0.03 mm，属于数值噪声；odom x≈0.000083 m，AMCL
为 `(0,0,0)`，三者一致。机械臂关节速度降至约 0.001 rad/s 以下。随后发送
`map:(0.3,0)` 导航目标，结果仍为 `SUCCEEDED`。

重复启动验收：在第一套 Gazebo 运行时再次执行相同 launch，立即得到清晰的
`Gazebo master 127.0.0.1:11345 is already in use` 提示，且没有生成第二个机器人、
Nav2 容器或 RViz。
