from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    
   # Launch file to launch the laserscan, localization and route_manager nodes simultaneously

    node1 = Node(
            package='amr_sanitizer_robot',
            executable='localize',
            name='localize',
            )
    node2 = Node(
            package='amr_sanitizer_robot',
            executable='laserscan',
            name='rdv',
            )
    node3 = Node(
            package='amr_sanitizer_robot',
            executable='RouteManager',
            name='RouteManager',
            )
   
   
    # Creation of the LaunchDescription
    ld = LaunchDescription()
    ld.add_action(node2)
    ld.add_action(node3)
    ld.add_action(node1)

    return ld
