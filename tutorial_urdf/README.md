```sh
sudo apt-get install ros-foxy-urdf-tutorial
source /opt/ros/foxy/setup.bash

mkdir -p dev_ws/urdf
touch urdf/01-myfirst.urdf
touch urdf/02-multipleshapes.urdf
touch urdf/03-origins.urdf
touch urdf/04-materials.urdf
touch urdf/05-visual.urdf

ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf
ros2 launch urdf_tutorial display.launch.py model:=urdf/02-multipleshapes.urdf
ros2 launch urdf_tutorial display.launch.py model:=urdf/03-origins.urdf
ros2 launch urdf_tutorial display.launch.py model:=urdf/04-materials.urdf
ros2 launch urdf_tutorial display.launch.py model:=urdf/05-visual.urdf
ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix --share urdf_tutorial`/urdf/06-flexible.urdf
ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix --share urdf_tutorial`/urdf/08-macroed.urdf.xacro
```

```sh
mkdir -p dev_ws/src
cd dev_ws/src
ros2 pkg create urdf_tutorial_r2d2 --build-type ament_python --dependencies rclpy
mkdir -p urdf_tutorial_r2d2/urdf
wget http://docs.ros.org/en/foxy/_downloads/71d2794a0272622b026635806c2739da/r2d2.urdf.xml -P urdf_tutorial_r2d2/urdf/
wget http://docs.ros.org/en/foxy/_downloads/da6b43a9c9a62bac7183458dc5a8a1ef/r2d2.rviz -P urdf_tutorial_r2d2/urdf/
touch urdf_tutorial_r2d2/urdf_tutorial_r2d2/state_publisher.py
mkdir -p urdf_tutorial_r2d2/launch
touch urdf_tutorial_r2d2/launch/demo.launch.py
cd ..
colcon build
. install/setup.bash
ros2 launch urdf_tutorial_r2d2 demo.launch.py
rviz2 -d install/urdf_tutorial_r2d2/share/urdf_tutorial_r2d2/r2d2.rviz

```
