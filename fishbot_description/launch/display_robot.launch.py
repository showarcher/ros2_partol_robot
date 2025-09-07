import launch
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory  # 根据包名获取安装后包的 share 目录
import os
import launch_ros.parameter_descriptions  # 提供 ParameterValue 用于封装节点参数，可支持延迟求值

def generate_launch_description():

    # 1. 获取 URDF 文件路径
    #    - get_package_share_directory('fishbot_description') 返回已安装包的 share 目录
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'fishbot/fishbot.urdf.xacro')
    

    # 2. 获取 RViz 配置 / Gazebo world世界文件路径
    default_rviz_path = os.path.join(urdf_package_path, 'config', 'fishbot_display.rviz')

    # 3. 声明 launch 参数 `model`
    #    - 用户可通过命令行传参 model:=<path> 指定 URDF 文件路径，默认使用 default_urdf_path
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_urdf_path),
        description="加载的机器人模型文件路径（URDF/XACRO）"
    )

    # 4. 读取 URDF/Xacro 文件内容
    #    - 使用 Command substitution 在 launch 执行时动态读取文件内容
    #    - LaunchConfiguration('model') 获取上面声明的参数值（支持被命令行覆盖）
    #    - 注意此处空格不能省略:'cat ' 或者'xacro '
    substitutions_command_result = launch.substitutions.Command(
    #   ['cat ', launch.substitutions.LaunchConfiguration('model')]
        ['xacro ', launch.substitutions.LaunchConfiguration('model')]
    )

    # 5. 将读取到的字符串包装为 ParameterValue 对象
    #    - 支持延迟求值，保证参数值是字符串类型（robot_description节点参数的要求）
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result,
        value_type=str
    )

    # 6. 启动 robot_state_publisher 节点
    #    - 解析 robot_description URDF，发布 /tf 坐标变换消息
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )

    # 7. rviz-启动 joint_state_publisher 节点
    #    - 发布关节状态 /joint_states，配合 robot_state_publisher 使用
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # 8. rviz-启动 RViz2，加载默认配置文件
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_path]
    )

    # 9. 返回 LaunchDescription，包含所有声明和启动动作
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,

        # 使用Rviz需要的actions：
        action_joint_state_publisher,
        action_rviz_node
    ])
