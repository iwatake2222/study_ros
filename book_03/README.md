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