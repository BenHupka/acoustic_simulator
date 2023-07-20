from ament_index_python.packages import get_package_share_path
import launch
import launch_ros
import os


def generate_launch_description():
    package_name = 'acoustic_simulator'
    package_path = get_package_share_path(package_name)
    default_depth_ekf_params_path = package_path / (
        'config/depth_ekf_params.yaml')
    default_atmospheric_pressure = '101300'

    default_vehicle_name = 'uuv00'

    atmospheric_pressure = launch.substitutions.LaunchConfiguration(
        'atmospheric_pressure')

    # atmospheric_pres
    launch_args = [
        launch.actions.DeclareLaunchArgument(
            name='vehicle_name',
            default_value=default_vehicle_name,
            description='Vehicle name used as top level namespace.',
        ),
        launch.actions.DeclareLaunchArgument(
            name='depth_ekf_params_path',
            default_value=str(default_depth_ekf_params_path),
            description='Path to the mixer configuration .yaml file.'),
        launch.actions.DeclareLaunchArgument(
            name='atmospheric_pressure',
            default_value=str(default_atmospheric_pressure),
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value=str(False),
        ),
    ]

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')

    depth_ekf_node = launch_ros.actions.Node(
        package=package_name,
        executable='depth_estimator_node',
        parameters=[
            launch.substitutions.LaunchConfiguration(
                'depth_ekf_params_path'),  # load yaml file with ekf parameters
            {
                'atmospheric_pressure': atmospheric_pressure,
            },
        ],
        output='screen',
        emulate_tty=True,
    )

    nodes_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace(
            launch.substitutions.LaunchConfiguration('vehicle_name')),
        depth_ekf_node,
    ])

    return launch.LaunchDescription(launch_args + [
        nodes_group,
    ])
