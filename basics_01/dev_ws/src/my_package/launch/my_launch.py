from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "my_parameter",
            default_value="I AM FIRST NODE. ",
            description="my parameter for First Node"
        ),
    ]
    nodes = [
        Node(
            package="my_package",
            executable="first",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "my_parameter": LaunchConfiguration("my_parameter"),
            }]
        ),
        Node(
            package="my_package",
            executable="second",
            output="screen",
            emulate_tty=True,
        ),
    ]
    return LaunchDescription(launch_args + nodes)

