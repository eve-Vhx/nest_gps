from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    input_id_arg = DeclareLaunchArgument(
        "input_id",
        default_value="NEST11010",
        description="Input ID for the drone"
    )

    # Configure the 'nest_gps_node' node
    nest_gps_node = Node(
        package="nest_gps",
        executable="nest_gps_node",
        output="screen",
        # parameters=[
        #     {"id": LaunchConfiguration("input_id")}
        # ]
    )

    # Create the launch description and populate it with the nodes and launch arguments
    ld = LaunchDescription()
    # ld.add_action(input_id_arg)
    ld.add_action(nest_gps_node)

    return ld
