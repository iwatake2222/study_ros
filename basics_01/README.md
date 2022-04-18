http://docs.ros.org/en/foxy/

# Setup
```sh
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

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
cd src
ros2 pkg create --build-type ament_cmake my_interface
cd my_interface
mkdir msg
mkdir srv
mkdir action
touch msg/Num.msg
touch srv/AddThreeInts.srv
touch action/Fibonacci.action
cd ../../
```

