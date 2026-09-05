import os
from glob import glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ROS setup does not source Gazebo Classic's setup.sh on Ubuntu.  Camera
    # and ray sensors still need rendering resources in server-only mode.
    gazebo_share_candidates = sorted(glob('/usr/share/gazebo-*'))
    if gazebo_share_candidates:
        gazebo_share_dir = gazebo_share_candidates[-1]
        resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
        resource_entries = [entry for entry in resource_path.split(':') if entry]
        if gazebo_share_dir not in resource_entries:
            os.environ['GAZEBO_RESOURCE_PATH'] = ':'.join(
                [gazebo_share_dir, *resource_entries])

        model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
        model_entries = [entry for entry in model_path.split(':') if entry]
        system_models = os.path.join(gazebo_share_dir, 'models')
        if system_models not in model_entries:
            os.environ['GAZEBO_MODEL_PATH'] = ':'.join(
                [system_models, *model_entries])

    ogre_candidates = sorted(
        path for path in glob('/usr/lib/*/OGRE-*')
        if os.path.isdir(path) and 'OGRE-Next' not in path)
    if ogre_candidates and not os.environ.get('OGRE_RESOURCE_PATH'):
        os.environ['OGRE_RESOURCE_PATH'] = ogre_candidates[-1]

    description_share = get_package_share_directory('robot_rl_description')
    gazebo_share = get_package_share_directory('gazebo_ros')

    # Migrated PAL worlds use model:// URIs.  Keep their model assets inside
    # this package and add that directory before Gazebo resolves the world.
    local_model_dir = os.path.join(description_share, 'models', 'pal')
    model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_entries = [entry for entry in model_path.split(':') if entry]
    if local_model_dir not in model_entries:
        os.environ['GAZEBO_MODEL_PATH'] = ':'.join(
            [local_model_dir, *model_entries])

    default_model = os.path.join(
        description_share, 'urdf', 'fishbot', 'fishbot.urdf.xacro')
    default_world = os.path.join(
        description_share, 'world', 'custom_room.world')

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    robot_description = ParameterValue(
        Command(['xacro ', model]), value_type=str)

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    ground_truth_odom_tf = Node(
        package='robot_rl_description',
        executable='ground_truth_odom_tf.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': verbose,
        }.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'fishbot',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
            '-timeout', '60.0',
        ],
    )

    controller_names = [
        'fishbot_joint_state_broadcaster',
        'fishbot_diff_drive_controller',
        'left_arm_controller',
        'right_arm_controller',
        'left_gripper_controller',
        'right_gripper_controller',
    ]
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                name,
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '60',
            ],
        )
        for name in controller_names
    ]

    actions = [
        DeclareLaunchArgument(
            'model', default_value=default_model,
            description='Absolute path to the robot URDF/Xacro file'),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Absolute path to the Gazebo Classic world file'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start gzclient (set false for headless/server-only mode)'),
        DeclareLaunchArgument(
            'verbose', default_value='true',
            description='Enable verbose Gazebo output'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0',
            description='Robot initial X coordinate in the Gazebo world'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0',
            description='Robot initial Y coordinate in the Gazebo world'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.002',
            description='Robot initial Z coordinate (wheels just touch ground)'),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0',
            description='Robot initial yaw angle in radians'),
        LogInfo(msg=['FishBot model: ', model]),
        LogInfo(msg=['Gazebo world: ', world]),
        state_publisher,
        ground_truth_odom_tf,
        gazebo,
        spawn_entity,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_entity,
            on_exit=[controller_spawners[0]],
        )),
    ]

    for current, following in zip(
            controller_spawners, controller_spawners[1:]):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=current,
            on_exit=[following],
        )))

    return LaunchDescription(actions)
