from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bringup',
            executable='bringup_node.py',
            name='bringup_node',
            output='screen'
        )
    ])
