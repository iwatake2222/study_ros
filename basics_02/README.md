http://docs.ros.org/en/foxy/

# Coding
```sh
mkdir -p dev_ws/src
cd dev_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp rclcpp_components std_msgs --node-name talker my_package
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
# . install/local_setup.bash
. install/setup.bash
ros2 run my_package talker
ros2 run my_package listener
ros2 launch my_package my_launch.py
```
