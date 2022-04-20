from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_package',
                    plugin='my_package::Talker',
                    name='talker'),
                ComposableNode(
                    package='my_package',
                    plugin='my_package::Listener',
                    name='listener')
            ],
            output='screen',
    )

    return LaunchDescription([container])