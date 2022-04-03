import launch.actions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        launch.actions.LogInfo(msg="Launch turtlesim node and turtle_teleop_key node."),
        launch.actions.TimerAction(period=3.0, actions=[launch.actions.LogInfo(msg="It's been three minutes since the launch.")]),
        Node(
            package="turtlesim",
            node_namespace="turtlesim",
            node_executable="turtlesim_node",
            node_name="turtlesim",
            output="screen",
            parameters=[{"background_r": 0}, {"background_g": 0}, {"background_b": 255},]
        ),
        Node(
            package="turtlesim",
            node_namespace="turtlesim",
            node_executable="turtle_teleop_key",
            node_name="teleop_turtle",
            prefix="xterm -e"
        ),
    ])