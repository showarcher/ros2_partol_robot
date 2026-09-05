# 仓库场景集成与测试记录

测试日期：2026-09-05。环境：本机 Ubuntu / ROS 2 Humble / Gazebo Classic，FishBot 模型。

## 集成方案

- 将上游 world、PGM/YAML、模型、网格与纹理完整放入现有 description/navigation2 包；不引入上游 TurtleBot3 包或绝对路径 launch。
- `scene:=warehouse` 默认选择配套仓库世界和地图，`scene:=room` 保留原房间；外部世界仍支持 `world` / `map` 参数。
- 仓库默认 AMCL 发布 `map→odom`；原房间可沿用固定真值变换；SLAM 模式自动关闭固定 map TF。
- 地板 mesh 顶面约为世界 Z=0.0342 m。仓库出生高度设为 0.08 m，防止沿用 0.002 m 时轮子嵌入地板。稳定后的 base_footprint 高度实测约 0.0362 m。
- 巡检按场景选择独立 YAML，保留自定义 `patrol_config` 参数。

资源版本与许可见 [第三方资源说明](third_party/README.md)。

## 检查结果

| 检查 | 结果与范围 |
| --- | --- |
| 编译 | `robot_rl_description`、`robot_rl_navigation2`、`autopatrol_robot` 编译通过 |
| 上游文件一致性 | 世界、地图及模型文件与固定上游提交逐字节一致 |
| 模型引用 | 67 处 `model://` 引用均在随包资源中存在 |
| 离线回归 | 3 项 unittest 通过：模型引用、地图与巡检点、场景切换及固定 TF 互斥 |
| 地图加载 | 318 × 439，0.05 m/像素，与原始仓库地图一致 |
| 传感器 | `/scan` 360 束，出生点有 262 束有限距离；相机 640 × 480 RGB |
| 导航生命周期 | map_server、amcl、planner_server、controller_server、bt_navigator 均 active |
| TF | 可查询 `map → base_link`；出生点位置接近 `(0, 0)` |
| 短距离规划 | 从出生点到 `(1, 0)`，21 个路径位姿，action 状态 SUCCEEDED |
| 实际移动 | NavigateToPose `(1, 0)` 返回 SUCCEEDED；真值 odom 最终约 `(0.883, -0.016)` |
| 四站规划 | 从出生点分别规划至四个站点全部成功，路径位姿数为 207 / 187 / 181 / 187 |
| GUI | Gazebo gzclient 启动，RViz 节点上线，运行检查期间无进程崩溃 |
| SLAM | `slam:=true` 启动成功，slam_toolbox 上线，生成 275 × 288 初始局部地图，TF 可查询 |
| 巡检入口 | launch 参数解析通过，新增仓库 YAML 已安装，全部目标落在地图空闲栅格 |

原始结果：[导航](testing/warehouse-navigation.json)、[GUI 与四站规划](testing/warehouse-gui.json)、[SLAM](testing/warehouse-slam.json)。

实际移动结果的真值 odom 与地图坐标不要求完全相等：AMCL 估计 `map→odom`，成功判定使用 map 坐标及目标容差。

## 测试方式与复查

测试使用独立的 `ROS_DOMAIN_ID=65`、`ROS_LOCALHOST_ONLY=1` 和 `GAZEBO_MASTER_URI=http://127.0.0.1:11365`。测试进程结束后关闭自己启动的 Gazebo/Nav2/RViz。

离线检查可在工作区根目录复跑：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover -s tests -v
```

加载与实际导航复查（每个终端加载相同环境）：

```bash
ros2 launch robot_rl_navigation2 navigation2.launch.py scene:=warehouse
```

等差速控制器和 Nav2 激活后，在另一终端检查并发送短距离目标：

```bash
ros2 control list_controllers
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 topic echo /map --once --field info
ros2 run tf2_ros tf2_echo map base_link
# 上条持续查询命令用 Ctrl+C 结束后，再发目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}' --feedback
```

## 已知边界

- 四站测试验证的是从出生点到各站的规划，不是整条四站任务的连续实际行驶。未实测无限循环巡检、语音播报或站点视觉识别。
- SLAM 测试验证启动、初始地图和 TF；未重新遍历整个仓库建图。
- FishBot 使用仿真真值里程计，不代表真实机器人编码器/IMU 定位性能。
- 本机原有资源会产生 `Gazebo/Silver` 材质警告、PAL `willowgarage` 的 model.config 警告；本次仓库资源加载与相机输出正常。退出时 Nav2 组件容器可能需要 launch 超时终止，不属于运行中崩溃。
