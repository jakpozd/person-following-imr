from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    detector_node = Node(
        package='pilot_detector',
        executable='detector_node',
        name='detector_node',
        output='screen',
    )
    follower_node = Node(
        package='pilot_detector',
        executable='follower_node',
        name='follower_node',
        output='screen',
    )

    return LaunchDescription([
        detector_node,
        follower_node
    ])
