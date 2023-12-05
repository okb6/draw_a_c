import os 

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('anyword'),
        'config',
        'alphabet.yaml'
    )


    anyword = Node(
        package="anyword",
        namespace="",
        executable="anyword",
        name="anyword",
        parameters=[config],
        output = 'screen'

    )

    ld.add_action(anyword)
    return ld