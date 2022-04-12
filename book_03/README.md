# Setup
http://wiki.ros.org/ja/noetic/Installation/Ubuntu

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y ros-noetic-rqt*
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update

source /opt/ros/noetic/setup.bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

roscore

rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
rqt_graph
```

```sh
mkdir -p catkin_ws/src && cd catkin_ws/src
catkin_init_workspace

catkin_create_pkg my_first_ros_pkg std_msgs roscpp
cd ..

catkin_make
source /home/iwatake/devel/study_ros/book_03/catkin_ws/devel/setup.bash
rospack profile

rosrun my_first_ros_pkg hello_world_node
rostopic echo /say_hello_world 
```

# Turtlebot
```sh
sudo apt install ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-depthimage-to-laserscan ros-noetic-rosserial-arduino ros-noetic-rosserial-python ros-noetic-rosserial-server ros-noetic-rosserial-client ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro ros-noetic-compressed-image-transport ros-noetic-rqt-image-view ros-noetic-gmapping ros-noetic-navigation  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-depthimage-to-laserscan ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro ros-noetic-compressed-image-transport ros-noetic-rqt-image-view ros-noetic-gmapping ros-noetic-navigation

cd catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations
cd ..
catkin_make
```

```sh
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
roslaunch turtlebot3_fake turtlebot3_fake.launch 
# export LDS_MODEL=LDS-01
# roslaunch turtlebot3_bringup turtlebot3_robot.launch --screen
# roslaunch turtlebot3_bringup turtlebot3_model.launch

# gazebo
roslaunch turtlebot3_gazebo turtlebot3_world.launch 
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch 
```

``sh
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
rosbag record -O scandata /scan /tf
roslaunch turtlebot3_slam turtlebot3_slam.launch
rosrun map_server map_saver -f ./maps
```

``sh
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=`pwd`/maps.yaml
```
