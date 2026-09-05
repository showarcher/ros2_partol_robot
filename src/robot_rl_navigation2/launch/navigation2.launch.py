import os
import socket
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def ensure_gazebo_master_is_free(context):
    """Fail before starting ROS nodes if another local Gazebo owns the master."""
    if LaunchConfiguration('start_gazebo').perform(context).lower() != 'true':
        return []

    master_uri = os.environ.get(
        'GAZEBO_MASTER_URI', 'http://127.0.0.1:11345')
    parsed = urlparse(master_uri)
    host = parsed.hostname or '127.0.0.1'
    port = parsed.port or 11345
    try:
        with socket.create_connection((host, port), timeout=0.3):
            raise RuntimeError(
                f'Gazebo master {host}:{port} is already in use. '
                'Close the previous ros2 launch/gzserver process, or launch '
                'with start_gazebo:=false to deliberately reuse it.')
    except (ConnectionRefusedError, TimeoutError, OSError):
        return []


def generate_launch_description():
    navigation_share = get_package_share_directory('robot_rl_navigation2')
    description_share = get_package_share_directory('robot_rl_description')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    scene = LaunchConfiguration('scene')
    default_map = PythonExpression([
        repr(os.path.join(navigation_share, 'maps', 'warehouse_map.yaml')),
        " if '", scene, "' == 'warehouse' else ",
        repr(os.path.join(navigation_share, 'maps', 'room.yaml')),
    ])
    default_params = os.path.join(
        navigation_share, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(
        navigation_share, 'config', 'fishbot_nav2.rviz')
    default_world = PythonExpression([
        repr(os.path.join(description_share, 'world', 'warehouse', 'small_warehouse.world')),
        " if '", scene, "' == 'warehouse' else ",
        repr(os.path.join(description_share, 'world', 'custom_room.world')),
    ])
    default_model = os.path.join(
        description_share, 'urdf', 'fishbot', 'fishbot.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_delay = LaunchConfiguration('rviz_delay')
    nav2_delay = LaunchConfiguration('nav2_delay')
    start_gazebo = LaunchConfiguration('start_gazebo')
    slam = LaunchConfiguration('slam')
    # nav2_bringup in Humble evaluates this argument in a PythonExpression
    # ("not <value>").  Normalize common ROS boolean spellings to Python's
    # capitalized True/False so slam:=true and the default both work.
    slam_for_nav2 = PythonExpression([
        "'", slam, "'.lower() == 'true'",
    ])
    map_path = LaunchConfiguration('map')
    params_path = LaunchConfiguration('nav2_params_file')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    # Exactly one authority for map -> odom: ground truth, AMCL, or SLAM.
    fixed_map_tf = PythonExpression([
        "'", LaunchConfiguration('localization'), "' == 'ground_truth' and '",
        slam, "'.lower() != 'true'",
    ])
    configured_params = RewrittenYaml(
        source_file=params_path,
        root_key='',
        param_rewrites={
            'amcl.ros__parameters.tf_broadcast': PythonExpression([
                "not (", fixed_map_tf, ")"]),
            'amcl.ros__parameters.initial_pose.x': LaunchConfiguration('initial_x'),
            'amcl.ros__parameters.initial_pose.y': LaunchConfiguration('initial_y'),
            'amcl.ros__parameters.initial_pose.yaw': LaunchConfiguration('initial_yaw'),
        },
        convert_types=True,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            description_share, 'launch', 'gazebo_sim.launch.py')),
        launch_arguments={
            'world': world,
            'publish_map_tf': fixed_map_tf,
            'model': model,
            'gui': gui,
            'verbose': verbose,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw,
        }.items(),
        condition=IfCondition(start_gazebo),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_path,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'slam': slam_for_nav2,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scene', default_value='warehouse', choices=['warehouse', 'room'],
            description='Select a matching built-in Gazebo world and navigation map'),
        DeclareLaunchArgument(
            'localization', choices=['amcl', 'ground_truth'],
            default_value=PythonExpression([
                "'amcl' if '", scene, "' == 'warehouse' else 'ground_truth'"]),
            description='AMCL for saved maps, or world-aligned ground truth for room demo'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo simulation clock'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 with the Nav2 default configuration'),
        DeclareLaunchArgument(
            'rviz_delay', default_value='15.0',
            description='Seconds to wait for Gazebo and AMCL before RViz starts'),
        DeclareLaunchArgument(
            'nav2_delay', default_value='8.0',
            description='Seconds to wait for Gazebo TF and sensors before Nav2 starts'),
        DeclareLaunchArgument(
            'start_gazebo', default_value='true',
            description='Start Gazebo, spawn FishBot and load its controllers'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='Run SLAM instead of AMCL localization with a saved map'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo client window'),
        DeclareLaunchArgument(
            'verbose', default_value='true',
            description='Enable verbose Gazebo output'),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Absolute path to the Gazebo world file'),
        DeclareLaunchArgument(
            'model', default_value=default_model,
            description='Absolute path to the robot URDF/Xacro file'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0',
            description='Robot initial X coordinate in the Gazebo world'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0',
            description='Robot initial Y coordinate in the Gazebo world'),
        DeclareLaunchArgument(
            'spawn_z', default_value=PythonExpression([
                "'0.08' if '", scene, "' == 'warehouse' else '0.002'"]),
            description='Spawn height: 0.08 clears warehouse floor; 0.002 for room'),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0',
            description='Robot initial yaw angle in radians'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Absolute path to the map YAML file'),
        DeclareLaunchArgument(
            'nav2_params_file', default_value=default_params,
            description='Absolute path to the Nav2 parameter file'),
        DeclareLaunchArgument('initial_x', default_value=spawn_x,
                             description='Initial X in map coordinates (not map YAML origin)'),
        DeclareLaunchArgument('initial_y', default_value=spawn_y,
                             description='Initial Y in map coordinates'),
        DeclareLaunchArgument('initial_yaw', default_value=spawn_yaw,
                             description='Initial heading in map coordinates, radians'),
        OpaqueFunction(function=ensure_gazebo_master_is_free),
        gazebo,
        TimerAction(period=nav2_delay, actions=[nav2]),
        TimerAction(period=rviz_delay, actions=[rviz]),
    ])
