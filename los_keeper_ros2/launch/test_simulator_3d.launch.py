import os
from launch import LaunchDescription
from launch_ros.actions import Node

current_directory = os.path.dirname(os.path.abspath(__file__))
parameters = [os.path.join(current_directory, os.pardir, 'config', 'parameters_3d.yaml')]


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='los_keeper_ros2',
            namespace='los_keeper',
            executable='los_server_node',
            name='los_server_node',
            output='screen',
            parameters=parameters,
            emulate_tty=True,
            remappings=[("~/state", "/drone_state"),
                        ("~/points", "/point_cloud"),
                        ("~/object_state_array",
                         "/obstacle_state_list"),
                        ("~/target_state_array",
                         "/target_state_list")]
        )
    ])
