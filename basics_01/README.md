http://docs.ros.org/en/foxy/

# Setup
## Local
```sh
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

## Docker
- https://docs.docker.com/engine/install/ubuntu/
- `sudo gpasswd -a iwatake docker`

```sh
xhost local:
docker run -it --name foxytest1 -e DISPLAY=$DISPLAY --net=host -v /dev:/dev -v /home/iwatake/devel/study_ros/:/study_ros --privileged ros:foxy
docker start foxytest1
docker exec -it foxytest1 /ros_entrypoint.sh bash

cd /study_ros
ros2 topic pub /chatter std_msgs/String "data: Hello World"
ros2 topic echo /chatter
```

# Coding
```sh
mkdir -p dev_ws/src
cd dev_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp rclcpp_components rclcpp_action std_msgs --node-name first_node my_package
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
# . install/local_setup.bash
. install/setup.bash
ros2 run my_package first
ros2 run my_package second
ros2 launch my_package my_launch.py
```

```sh
cd src    - Install ms-vscode.cpptools, ms-python.python
touch msg/Num.msg
touch srv/AddThreeInts.srv
touch action/Fibonacci.action
cd ../../
```

# Development tools
## How to automatically set IntelliSense in VSCode
- VSCode
    - ms-vscode.cpptools
- Modify `CMakeLists.txt` to add
    - `set(CMAKE_EXPORT_COMPILE_COMMANDS ON)`
- Create `.vscode/c_cpp_properties.json` to add
    - `"compileCommands": "${workspaceFolder}/dev_ws/build/compile_commands.json",`

## How to Debug in VSCode
- Open VSCode after doing
    - `source /opt/ros/foxy/setup.bash`
    - ` . install/setup.bash`

- VSCode
    - ms-iot.vscode-ros
- Create `.vscode/launch.json` to add
    ```json
    {
        "version": "0.2.0",
        "configurations": [
            {
                "name": "ROS: Launch",
                "type": "ros",
                "request": "launch",
                "target": "${workspaceFolder}/dev_ws/src/my_package/launch/my_launch.py"
            }
        ]
    }
    ```
- Build with debug info
    ```sh
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
    ```
- F5


# Development tools for Docker
- Open `code` in local
- Remote Explorer -> Containers -> ros:foxy foxytest1 -> Open `/study_ros/basics_01`
    - Install ms-vscode.cpptools, ms-python.python
- Enable Python IntelliSense in VSCode
    - `.vscode/settings.json`

    ```json:.vscode/settings.json
    {
        "python.autoComplete.extraPaths": [
            "/opt/ros/foxy/lib/python3.8/site-packages/"
        ],
        "python.analysis.extraPaths": [
            "/opt/ros/foxy/lib/python3.8/site-packages/"
        ],
    }
    ```
- Enable C++ IntelliSense in VSCode
    - See above
- How to debug C++
    - Install GDB
    - File -> Preferences -> Settings -> Ros
        - `Ros: Distro` = `/opt/ros/foxy`
        - `Ros: Ros Setup Script` = `/study_ros/basics_01/dev_ws/install/setup.bash`
    - F5

