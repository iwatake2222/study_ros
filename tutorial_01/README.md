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
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
# . install/local_setup.bash
. install/setup.bash
ros2 run cpp_pubsub talker
ros2 run cpp_pubsub listener
```


