from ament_index_python.packages import get_package_share_path
from hippo_common.launch_helper import (
    LaunchArgsDict,
    declare_vehicle_name_and_sim_time,
)
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(launch_description=launch_description)

    default_acoustic_config_path = get_package_share_path(
        'acoustic_simulator') / 'config/modem_params.yaml'
    action = DeclareLaunchArgument(
        name='acoustic_config_path',
        default_value=str(default_acoustic_config_path),
        description='Path to the acoustic simulation configuration .yaml file',
    )
    launch_description.add_action(action)


def create_anchor_sim_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='acoustic_simulator',
        executable='sim_anchor_measurements_node',
        parameters=[
            args,
            LaunchConfiguration('acoustic_config_path'),
        ],
        output='screen',
        emulate_tty=True,
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)

    nodes_group = GroupAction([
        PushRosNamespace(LaunchConfiguration("vehicle_name")),
        create_anchor_sim_node()
    ])

    launch_description.add_action(nodes_group)

    return launch_description
