from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from hippo_common.launch_helper import declare_use_sim_time


def declare_launch_args(launch_description: LaunchDescription):
    declare_use_sim_time(launch_description=launch_description)

    # default_depth_ekf_params_path = get_package_share_path(
    #     'acoustic_simulator') / 'config/depth_ekf_params.yaml'
    # action = DeclareLaunchArgument(
    #     name='depth_ekf_params_path',
    #     default_value=str(default_depth_ekf_params_path),
    #     description='Path to mixer configuration .yaml file.')
    # launch_description.add_action(action)

    # action = DeclareLaunchArgument(
    #     name='atmospheric_pressure',
    #     default_value='101300',
    #     description='Atmospheric pressure used for depth estimation')
    # launch_description.add_action(action)


def add_node(launch_description: LaunchDescription):
    action = Node(
        package='acoustic_simulator',
        executable='tester_node',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
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
