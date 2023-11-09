from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import ReplaceString


def generate_launch_description():
    declared_arguments = []

    # Declare arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='palletron1',
            description='Top-level namespace.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'config_file',
            default_value='minimal_robot_view.rviz',
            description='Name of the Rviz configuration file to use.'))

    # Initialize arguments
    namespace = LaunchConfiguration('namespace')
    config_file = LaunchConfiguration('config_file')

    # Get full path to the Rviz configuration file
    config_file_path = PathJoinSubstitution([FindPackageShare('palletron_visualization'),
                                             'rviz',
                                             config_file])

    # Replace robot namespace in Rviz file
    namespaced_config_file = ReplaceString(
        source_file=config_file_path,
        replacements={'<robot_namespace>': ('/', namespace)})

    # Declare nodes
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', namespaced_config_file])

    return LaunchDescription(declared_arguments + [rviz2_node])
