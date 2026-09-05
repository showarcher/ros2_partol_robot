import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    autopatrol_robot_path = get_package_share_directory('autopatrol_robot')
    scene = launch.substitutions.LaunchConfiguration('scene')
    default_patrol_config_path = launch.substitutions.PythonExpression([
        repr(os.path.join(autopatrol_robot_path, 'config', 'warehouse_patrol.yaml')),
        " if '", scene, "' == 'warehouse' else ",
        repr(os.path.join(autopatrol_robot_path, 'config', 'patrol_config.yaml')),
    ])
    
    action_patrol_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='patrol_node',
        name='patrol_node',
        output='screen',
        parameters=[launch.substitutions.LaunchConfiguration('patrol_config')]
    )
    action_speaker_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='speaker',
        name='speaker',
        output='screen',
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'scene', default_value='warehouse', choices=['warehouse', 'room']),
        launch.actions.DeclareLaunchArgument(
            'patrol_config', default_value=default_patrol_config_path,
            description='YAML patrol points in the selected map coordinates'),
        action_patrol_node,
        action_speaker_node,
    ])
