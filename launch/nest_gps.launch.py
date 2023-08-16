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
        executable='nest_gps_node',
        name='nest_gps',
        namespace=LaunchConfiguration('namespace'),   # Using the argument
        output='screen')
    
    nest_sync_node = Node(
        package='nest_gps',  # Name of your package
        executable='nest_sync_node',  # Name of your node executable
        name='nest_sync_node',
        namespace=LaunchConfiguration('namespace'),   # Using the argument
        output="screen",

        # parameters=[
        #     {'id': 'default_topic_name'}  # Default topic name if none is provided
        # ],
    )
    # Include the actions in the launch description
    return LaunchDescription([
        declare_namespace_arg,
        nest_gps_node,
        nest_sync_node
    ])