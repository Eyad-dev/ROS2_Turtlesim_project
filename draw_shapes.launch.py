from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2_project",
            executable="shape_node",
            name="shape_node",
            output="screen"
        ),
        Node(
            package="ros2_project",
            executable="turtle_commander",
            name="turtle_commander",
            output="screen"
        ),
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim_node",
            output="screen"
        )
    ])
