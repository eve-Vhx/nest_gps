from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',   # Name of the argument
        default_value='nest01',   # Default value
        description='Namespace for the mission node')   # Description (optional)

    # Use the argument in the Node action
    nest_gps_node = Node(
        package='nest_gps',
        executable='nest_gps_t.py',
        name='nest_gps',
        namespace=LaunchConfiguration('namespace'),   # Using the argument
        output='screen')
    

    return LaunchDescription([
        declare_namespace_arg,
        nest_gps_node,
        ])
