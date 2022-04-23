import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('my_package'),
        'rviz',
        'pointcloud2_rviz.rviz'
    )

    print(rviz_config)

    return LaunchDescription([
        Node(
            package="my_package",
            executable="pointcloud2_talker_exe",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
        # Node(
        #     package="my_package",
        #     executable="my_listener_exe",
        #     output="screen",
        #     emulate_tty=True,
        # ),
    ])
