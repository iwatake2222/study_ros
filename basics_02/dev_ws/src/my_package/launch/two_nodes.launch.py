from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="my_package",
            executable="talker_exe",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="my_package",
            executable="listener_exe",
            output="screen",
            emulate_tty=True,
        ),
    ])
