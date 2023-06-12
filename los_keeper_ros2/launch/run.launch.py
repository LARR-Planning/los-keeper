import os
from launch import LaunchDescription
from launch_ros.actions import Node

current_directory = os.path.dirname(os.path.abspath(__file__))
parameters = [os.path.join(current_directory, os.pardir, 'config', 'parameters.yaml')]


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='los_keeper_ros2',
            namespace='los_keeper',
            executable='los_server_node',
            name='los_server_node',
            output='screen',
            parameters=parameters,
            emulate_tty=True
        ),
        Node(
            package='los_keeper_ros2',
            namespace='los_keeper',
            executable='publisher_test_node',
            name='test_publisher_node',
            output={'both': 'log'},
            remappings=[("~/state", "/los_keeper/los_server_node/state"),
                        ("~/points", "/los_keeper/los_server_node/points"),
                        ("~/target_state_array", 
                         "/los_keeper/los_server_node/target_state_array")]
        )
    ])
