import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(package="trial_parameter", executable="param_publisher", output="screen", emulate_tty=True, parameters=[{"parameter": "Tokyo Publisher"}]),
    ])