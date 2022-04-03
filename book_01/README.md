```sh
docker build -t iwatake/ros2 -f ros2.dockerfile .
# docker run -it -e DISPLAY="192.168.1.2:0.0" --rm -v `pwd`/myProjects:/root/myProjects -w /root iwatake/ros2
docker create -it -e DISPLAY="192.168.1.2:0.0" -v `pwd`/myProjects:/root/myProjects -w /root --name=ros2 iwatake/ros2
docker start ros2
docker exec -it ros2 bash
```

```sh
ros2 run rqt_plot rqt_plot
ros2 run rviz2 rviz2
gazebo --verbose /opt/ros/eloquent/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
# wait
ros2 topic pub /demo/cmd_demo geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: -0}}" -1
```

```sh
ros2 run demo_nodes_py talker
ros2 topic list -t
ros2 interface show std_msgs/msg/String
ros2 topic info /chatter
ros2 topic echo /chatter
```

```sh
ros2 run demo_nodes_py listener
ros2 topic list -t
ros2 interface show std_msgs/msg/String
ros2 topic pub /chatter std_msgs/msg/String "{data: Hello World}"
```

```sh
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
ros2 node list
ros2 node info /turtlesim
ros2 node info /teleop_turtle

ros2 run turtlesim  turtlesim_node --ros-args --remap __node:=my_turtle

rqt_graph

ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```


```sh
ros2 service list t
ros2 interface show std_srvs/srv/Empty
ros2 interface show turtlesim/srv/Spawn

ros2 service call /clear std_srvs/srv/Empty
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: 'ABC'}"
ros2 run turtlesim turtle_teleop_key /turtle1/cmd_vel:=/ABC/cmd_vel
```

```sh
ros2 param list /turtlesim
ros2 param get /turtlesim background_b
ros2 param set /turtlesim background_b 150
ros2 param dump /turtlesim
cat ./turtlesim.yaml
ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml
```


```sh
ros2 action list -t
ros2 interface show turtlesim/action/RotateAbsolute
ros2 action send_goal --feedback /turtle1/rotate_absolute turtlesim/action/RotateAbsolute {"theta: 1.57"}
```

```sh
ros2 launch ./myProjects/my_turtlesim.launch.py
```



# Chapter 8
```sh
ros2 run examples_rclpy_minimal_publisher publisher_local_function
ros2 run examples_rclpy_minimal_subscriber subscriber_lambda
```

```sh
cd ~/ros2_ws/src/myProjects
ros2 pkg create your_turtle_pkg --dependencies rclcpy --build-type ament_python
cd ~/ros2_ws
colcon build --packages-select your_turtle_pkg
source ~/ros2_ws/install/setup.bash

ros2 run your_turtle_pkg move
ros2 run your_turtle_pkg spawn
ros2 run your_turtle_pkg bg_param
ros2 run your_turtle_pkg rotate

```
