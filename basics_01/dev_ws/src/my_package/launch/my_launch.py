from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="my_package",
            executable="first",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "I AM FRST NODE. "}
            ]
        ),
        Node(
            package="my_package",
            executable="second",
            output="screen",
            emulate_tty=True,
        ),
    ])
