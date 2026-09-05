# robot_rl_description

## 启动与查看

RViz 展示:

```bash
ros2 launch robot_rl_description display_robot.launch.py
```

Gazebo Classic 仿真:

```bash
ros2 launch robot_rl_description gazebo_sim.launch.py
```

查看新增机械臂和机械爪关节状态:

```bash
ros2 topic echo /joint_states
ros2 control list_controllers
ros2 control list_joints
```

## 机械臂控制示例

左臂关节:

- `left_shoulder_joint`
- `left_elbow_joint`
- `left_wrist_joint`

右臂关节:

- `right_shoulder_joint`
- `right_elbow_joint`
- `right_wrist_joint`

左爪和右爪各有 9 个关节，命名格式为:

- `{left|right}_finger{1|2|3}_base_joint`
- `{left|right}_finger{1|2|3}_middle_joint`
- `{left|right}_finger{1|2|3}_tip_joint`

发送一个轻微移动左臂的轨迹目标:

```bash
ros2 action send_goal /left_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [left_shoulder_joint, left_elbow_joint, left_wrist_joint], points: [{positions: [0.25, 0.15, -0.10], time_from_start: {sec: 2}}]}}"
```

底盘差速控制仍使用 `/cmd_vel`。
