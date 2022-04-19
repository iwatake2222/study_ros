http://docs.ros.org/en/foxy/Tutorials/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html

```sh
source /opt/ros/foxy/setup.bash

cd devel/study_ros/
mkdir -p tutorial_tf/dev_ws/src
cd tutorial_tf/dev_ws/src
ros2 pkg create --build-type ament_cmake learning_tf2_cpp
cd ..

touch src/learning_tf2_cpp/src/static_turtle_tf2_broadcaster.cpp
```

```sh
ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0
ros2 topic echo --qos-reliability reliable --qos-durability transient_local /tf_static
```

```sh
# ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
ros2 run tf2_ros static_transform_publisher 0 0 1 0 0 0 world mystaticturtle

mkdir src/learning_tf2_cpp/launch
touch src/learning_tf2_cpp/launch/static_transform_publisher_launch.py
```


```sh
touch src/learning_tf2_cpp/src/turtle_tf2_broadcaster.cpp
touch src/learning_tf2_cpp/launch/turtle_tf2_demo.launch.py
touch src/learning_tf2_cpp/src/turtle_tf2_listener.cpp

ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py 
ros2 run turtlesim turtle_teleop_key 
ros2 run tf2_ros tf2_echo world turtle1
ros2 run tf2_tools view_frames.py
```

```sh
touch src/learning_tf2_cpp/src/fixed_frame_tf2_broadcaster.cpp
touch src/learning_tf2_cpp/src/dynamic_frame_tf2_broadcaster.cpp
touch src/learning_tf2_cpp/launch/turtle_tf2_fixed_frame_demo.launch.py
touch src/learning_tf2_cpp/launch/turtle_tf2_dynamic_frame_demo.launch.py
ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo.launch.py
ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo.launch.py target_frame:=carrot1
ros2 run tf2_tools view_frames.py
```