from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import PushRosNamespace
from hippo_common.launch_helper import PassLaunchArguments


def declare_launch_args(launch_description: LaunchDescription):
    # declare_vehicle_name_and_sim_time(launch_description=launch_description)
    pass


def include_launch_files(launch_description: LaunchDescription):
    package_path = get_package_share_path('acoustic_simulator')

    path = str(package_path / 'launch/node_tester.launch.py')
    source = PythonLaunchDescriptionSource(path)

    args = PassLaunchArguments()
    args.add_vehicle_name_and_sim_time()

    tester = IncludeLaunchDescription(source, launch_arguments=args.items())

    action = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        tester,
    ])
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    include_launch_files(launch_description=launch_description)

    return launch_description
