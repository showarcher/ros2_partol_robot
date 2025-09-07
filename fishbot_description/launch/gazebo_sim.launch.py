import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

# -----------------------------
# 0. 设置 Gazebo 使用本机网络，避免 multicast 错误
# -----------------------------
os.environ['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:11345'
os.environ['GAZEBO_IP'] = '127.0.0.1'

def generate_launch_description():
    # -----------------------------
    # 1. 获取默认路径
    # -----------------------------
    robot_name_in_model = "fishbot"
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_model_path = os.path.join(urdf_package_path, 'urdf', 'fishbot/fishbot.urdf.xacro')
    default_world_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')
    controller_yaml_path = os.path.join(urdf_package_path, 'config', 'fishbot_ros2_controller.yaml')

    # -----------------------------
    # 2. 为 Launch 声明参数
    # -----------------------------
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='URDF 文件的绝对路径'
    )

    # -----------------------------
    # 3. 读取 URDF/Xacro 文件内容
    # -----------------------------
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )

    # -----------------------------
    # 4. 启动 robot_state_publisher 节点
    # -----------------------------
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # -----------------------------
    # 5. 启动 ros2_control_node
    # -----------------------------
    ros2_control_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controller_yaml_path],
        output='screen'
    )

    # -----------------------------
    # 6. 启动 Gazebo 仿真
    # -----------------------------
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments=[('world', default_world_path), ('verbose', 'true')]
    )

    # -----------------------------
    # 7. spawn 机器人到 Gazebo
    # -----------------------------
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model],
        output='screen'
    )

    # -------------------------------------------------------------------------------------------------------
    # 8. 加载并激活 fishbot_joint_state_broadcaster 状态发布控制器(cmd指令，可以通过 cmd='指令'.spilt(' ') 的形式代替)
    # -------------------------------------------------------------------------------------------------------
    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fishbot_joint_state_broadcaster'],
        output='screen'
    )

    # ---------------------------------------------
    # 9. 加载并激活 fishbot_effort_controller 力控制器
    # ---------------------------------------------
    # action_load_fishbot_effort_controller = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_effort_controller'], 
    #     output='screen')
    
    # -----------------------------
    # 10. 加载差速控制器
    # -----------------------------
    action_load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fishbot_diff_drive_controller'],
        output='screen'
    )

    # -----------------------------
    # 11. 返回 LaunchDescription
    #    - 按事件顺序加载控制器
    # -----------------------------
    return launch.LaunchDescription([
        action_declare_arg_model_path,
        robot_state_publisher_node,
        ros2_control_node,
        launch_gazebo,
        action_spawn_entity,
        # 8.1 当机器人 spawn 完成后，激活 fishbot_joint_state_broadcaster 状态发布控制器
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller],
            )
        ),
        # 9.1 当激活状态发布控制器后，激活 fishbot_effort_controller 力控制器
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=action_spawn_entity,
        #         on_exit=[action_load_fishbot_effort_controller],
        #     )
        # ),

        # 10.1 当激活状态发布控制器后，激活 diff_drive_controller
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_diff_drive_controller],
            )
        ),
    ])
