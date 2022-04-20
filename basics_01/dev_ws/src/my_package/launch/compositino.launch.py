from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

'''
ros2 launch my_package compositino.launch.py my_parameter:="Modified Name." 
'''


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "my_parameter",
            default_value="I AM FIRST NODE. ",
            description="my parameter for First Node"
        ),
    ]
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_package',
                    plugin='my_package::FirstNode',
                    name='first',
                    parameters=[{
                        "my_parameter": LaunchConfiguration("my_parameter"),
                    }]
                ),
                ComposableNode(
                    package='my_package',
                    plugin='my_package::SecondNode',
                    name='second'
                )
            ],
            output='screen',
    )

    return LaunchDescription(launch_args + [container])