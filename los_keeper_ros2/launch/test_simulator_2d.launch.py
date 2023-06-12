import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


# current_directory = os.path.dirname(os.path.abspath(__file__))
# parameters = [os.path.join(current_directory, os.pardir, 'config', 'parameters_2d.yaml')]


def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('los_keeper_ros2'),
            'config',
            'parameters_2d.yaml')
    )
    DeclareLaunchArgument(
        'param_dir',
        default_value=param_dir,
        description='YAML FILE',
    )
    planner_node = Node(
        package='los_keeper_ros2',
        executable='los_server_node',
        namespace='los_keeper',
        name='los_server_node',
        output='screen',
        parameters=[param_dir],
        emulate_tty=True,
        remappings=[("~/state", "/drone_state"),
                    ("~/points", "/point_cloud"),
                    ("~/object_state_array",
                     "/obstacle_state_list"),
                    ("~/target_state_array",
                     "/target_state_list")]
    )
    return LaunchDescription([
        planner_node,
    ])
