from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nest_gps',
            namespace='nest01',
            executable='nest_gps_node',
            output='screen'
        )
    ])