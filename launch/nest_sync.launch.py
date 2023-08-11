from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    input_id_arg = DeclareLaunchArgument(
        "id",
        default_value="NEST11_0_1_0",
        description="Input ID for the drone"
    )

    # Configure the 'nest_gps_node' node
    nest_sync_node = Node(
        package='nest_gps',  # Name of your package
        executable='nest_sync_node',  # Name of your node executable
        name='nest_sync_node',
        output="screen",

        # parameters=[
        #     {'id': 'default_topic_name'}  # Default topic name if none is provided
        # ],
    )

    # Create the launch description and populate it with the nodes and launch arguments
    ld = LaunchDescription()
    # ld.add_action(input_id_arg)
    ld.add_action(nest_sync_node)

    return ld
