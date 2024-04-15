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
        'acoustic_simulator') / 'config/acoustic_params.yaml'
    action = DeclareLaunchArgument(
        name='acoustic_config_path',
        default_value=str(default_acoustic_config_path),
        description='Path to the acoustic simulation configuration .yaml file',
    )
    launch_description.add_action(action)

    default_vehicle_modem_config_file = get_package_share_path(
        'acoustic_simulator') / 'config/modem_bluerov_default.yaml'
    action = DeclareLaunchArgument(
        name='tf_vehicle_modem_config_file',
        description='TF config for agent modem file',
        default_value=str(default_vehicle_modem_config_file))
    launch_description.add_action(action)


def create_anchor_sim_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='acoustic_simulator',
        executable='sim_anchor_measurements_node.py',
        parameters=[
            args,
            LaunchConfiguration('acoustic_config_path'),
        ],
        output='screen',
        emulate_tty=True,
    )


def create_anchor_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='acoustic_simulator',
        executable='anchor_poses_node.py',
        parameters=[
            args,
            LaunchConfiguration('acoustic_config_path'),
        ],
        output='screen',
        emulate_tty=True,
    )


def create_ground_truth_distance_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='acoustic_simulator',
        executable='ground_truth_distance_node.py',
        parameters=[
            args,
            LaunchConfiguration('acoustic_config_path'),
        ],
        output='screen',
        emulate_tty=True,
    )


def create_rviz_robot_mesh_publisher():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='hippo_common',
        executable='rviz_robot_mesh_publisher',
        output='screen',
        parameters=[
            args,
        ],
    )


def create_rviz_node():
    rviz_file = str(
        get_package_share_path('acoustic_simulator') / 'config/rviz.rviz')
    return Node(
        executable='rviz2',
        package='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file, '--ros-args', '--log-level', 'error'],
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)

    nodes_group = GroupAction([
        PushRosNamespace(LaunchConfiguration("vehicle_name")),
        create_anchor_sim_node(),
        create_ground_truth_distance_node(),
        create_anchor_node(),
        create_rviz_robot_mesh_publisher(),
        create_rviz_node()
    ])

    launch_description.add_action(nodes_group)

    return launch_description
