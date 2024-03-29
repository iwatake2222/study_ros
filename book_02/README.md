# Setup
- https://docs.docker.com/engine/install/ubuntu/

```sh
sudo gpasswd -a iwatake docker
xhost local:
docker run -it --rm --name foxytest1 -e DISPLAY=$DISPLAY --net=host -v /dev:/dev --privileged ros:foxy
```

# Test by console command
```sh
xhost local:
docker run -it --rm --name foxytest1 -e DISPLAY=$DISPLAY --net=host -v /dev:/dev -v /home/iwatake/iwatake/devel/study_ros/:/study_ros --privileged ros:foxy
docker exec -it foxytest1 /ros_entrypoint.sh bash
ros2 topic pub /chatter std_msgs/String "data: Hello World"
ros2 topic echo /chatter
```

# Create topic node
```sh
mkdir -p my_ws/src && cd my_ws/src
ros2 pkg create --build-type ament_python my_pkg_topic
cd ..

touch src/my_pkg_topic/my_pkg_topic/my_publisher.py src/my_pkg_topic/my_pkg_topic/my_subscriber.py
nano src/my_pkg_topic/setup.py
colcon build
. ./install/setup.bash
ros2 run my_pkg_topic publisher
ros2 run my_pkg_topic subscriber
```

- Modify code
    - code -> Remote Explorer -> Container
    - Install ms-python.python Extention
    - ctrl + shift + p -> Change language mode -> Configure `python` based language settings. (settings.json)

```txt:settings.json
{
    "editor.renderWhitespace": "all",
    "window.zoomLevel": -1,
    "[python]": {
        "editor.wordBasedSuggestions": false
    },
    "python.analysis.extraPaths": [
        "/opt/ros/foxy/lib/python3.8/site-packages/"
    ],
    "python.autoComplete.extraPaths": [
        "/opt/ros/foxy/lib/python3.8/site-packages/"
    ],
}
```

```txt:src/my_pkg_topic/setup.py
    entry_points={
        'console_scripts': [
            'publisher = my_pkg_topic.my_publisher:main',
            'subscriber = my_pkg_topic.my_subscriber:main',
        ],
    },
```

# Create launch
```sh
cd my_ws/src
ros2 pkg create --build-type ament_python my_pkg_launch
cd ..

mkdir -p src/my_pkg_launch/launch
touch src/my_pkg_launch/launch/my_launch.launch.py
nano src/my_pkg_launch/setup.py

colcon build
. ./install/setup.bash
ros2 launch my_pkg_launch my_launch.launch.py 
```

# micro-ROS
```sh
docker run -it --name es32freertos --net=host -v /dev:/dev -v /home/iwatake/iwatake/devel/study_ros/:/study_ros --privileged ros:foxy
cd /study_ros/book_02/
mkdir -p esp32freertos_ws/src
git submodule add -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git esp32freertos_ws/src/micro_ros_setup

cd esp32freertos_ws
sudo apt update && rosdep update
rosdep install --from-path src/ --ignore-src -y
sudo apt install python3-pip
colcon build
source install/local_setup.bash

ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

ros2 run micro_ros_setup configure_firmware.sh int32_publisher -t udp -i 192.168.1.13 -p 8888
# Modify WiFi Configuration
ros2 run micro_ros_setup build_firmware.sh menuconfig
ros2 ros micro_ros_setup build_firmware.sh
ros2 run micro_ros_setup flash_firmware.sh

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
ros2 run micor_ros_agent micro_ros_agent udp4 --port 8888
# docker run -it --rm --net=host microros/micro-ros-agent:foxy udp4 --port 8888 -v6

ros2 run micro_ros_setup build_firmware.sh monitor
ros2 topic list
```

