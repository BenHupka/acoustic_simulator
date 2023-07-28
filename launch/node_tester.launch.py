from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from hippo_common.launch_helper import declare_use_sim_time


def declare_launch_args(launch_description: LaunchDescription):
    declare_use_sim_time(launch_description=launch_description)

    default_acoustic_config_path = get_package_share_path(
        'acoustic_simulator') / 'config/acoustic_config.json'
    action = DeclareLaunchArgument(
        name='acoustic_config_path',
        default_value=str(default_acoustic_config_path),
        description='Path to acoustic configuration .json file.')
    launch_description.add_action(action)


def add_node(launch_description: LaunchDescription):
    action = Node(
        package='acoustic_simulator',
        executable='tester_node',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            },
            {
                'acoustic_config_path':
                LaunchConfiguration('acoustic_config_path')
            },
        ],
        output='screen',
        emulate_tty=True,
    )
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    add_node(launch_description=launch_description)
    return launch_description
