import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    rviz_path = str(os.path.join(get_package_share_directory("los_keeper_simulator"), 'rviz_config', '3d.rviz'))
    initial_state_file_name_str = str(os.path.join(get_package_share_directory("los_keeper_simulator"), 'world', 'initial_state_3d_sc1.txt'))
    object_trajectory_file_name_str = str(os.path.join(get_package_share_directory("los_keeper_simulator"), 'world', 'object_trajectory_3d_sc1.csv'))
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('los_keeper_simulator'),
            'param',
            'sim_3d_sc1.param.yaml')
    )
    DeclareLaunchArgument(
        'param_dir',
        default_value=param_dir,
        description='YAML FILE',
    )

    simulator_node = Node(
        package='los_keeper_simulator',
        executable='los_keeper_simulator_node',
        parameters=[param_dir, {"initial_state_file_name": initial_state_file_name_str}, {"object_trajectory_file_name": object_trajectory_file_name_str} ],
        arguments=[],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_path]
    )
    return LaunchDescription(
        [
            simulator_node,
            rviz_node,
        ]
    )
