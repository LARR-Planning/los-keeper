from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='los_keeper_ros2',
            namespace='los_keeper',
            executable='los_server_node',
            name='los_server_node'
        ),
        Node(
            package='los_keeper_ros2',
            namespace='los_keeper',
            executable='publisher_test_node',
            name='test_publisher_node',
            output={'both': 'log'},
            remappings=[("~/state", "/los_keeper/los_server_node/state"),
                        ("~/points", "/los_keeper/los_server_node/points")]
        )
    ])
