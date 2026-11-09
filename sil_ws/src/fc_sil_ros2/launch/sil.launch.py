import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('fc_sil_ros2')
    world_file = os.path.join(share_dir, 'worlds', 'hexacopter_world.sdf')
    bridge_file = os.path.join(share_dir, 'config', 'bridge.yaml')
    params_file = os.path.join(share_dir, 'config', 'sil_params.yaml')

    model_resource_path = os.path.join(share_dir, 'models')
    old_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = model_resource_path
    if old_resource_path:
        resource_path += os.pathsep + old_resource_path

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': resource_path},
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fc_gz_bridge',
        parameters=[{'config_file': bridge_file}],
        output='screen',
    )

    flight_controller = Node(
        package='fc_sil_ros2',
        executable='fc_sil_node',
        name='fc_sil_node',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([gazebo, bridge, flight_controller])
