import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(package="trial_topic", executable="publisher", output="screen"),
        launch_ros.actions.Node(package="trial_topic", executable="subscriber", output="screen")
    ])