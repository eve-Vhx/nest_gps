from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',   # Name of the argument
        default_value='nest01',   # Default value
        description='Namespace for the mission node')   # Description (optional)



    # Configure the 'nest_gps_node' node
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

    # Create the launch description and populate it with the nodes and launch arguments
    ld = LaunchDescription([        
    declare_namespace_arg,
    nest_sync_node
    ])
    # ld.add_action(input_id_arg)
    # ld.add_action(nest_sync_node)

    return ld
