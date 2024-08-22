from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_pcan',
            executable='ros2pcan_node',
            name='ros2pcan_node'
        ),
        Node(
            package='jetracer2ros',
            executable='jetracer_ros_node',
	    name='jetracer_ros_node'
        ),
    ])
