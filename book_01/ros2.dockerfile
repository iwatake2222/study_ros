### Base image ###
FROM ubuntu:18.04

### Information ###
LABEL maintainer="iwatake <take.iwiw2222@gmail.com>"
ENV TZ JST-9
# SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

### Install basic packages ###
RUN apt-get update -q \
&& apt-get upgrade -yq \
&& apt-get install -yq bash-completion build-essential curl gnupg2 lsb-release locales git tmux wget nano gedit x11-apps eog locales git tmux wget nano gedit x11-apps eog \
&& rm -rf /var/lib/apt/lists/*

### Install ROS2 Eloquent ###
RUN curl -Ls https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt-get update -q
RUN apt-get install -yq ros-eloquent-desktop
### Install ROS2 Eloquent tools ###
RUN apt-get install -yq python3-argcomplete python3-colcon-common-extensions python3-vcstool

### Install Turtlesim ###
RUN apt-get install -yq ros-eloquent-turtlesim ros-eloquent-gazebo-ros-*

### Install Turtlebot3 ###
# Install cartographer #
RUN apt-get install -yq google-mock libceres-dev liblua5.3-dev libboost-dev libboost-iostreams-dev libprotobuf-dev protobuf-compiler libcairo2-dev libpcl-dev python3-sphinx
RUN apt-get install -yq ros-eloquent-cartographer ros-eloquent-cartographer-ros ros-eloquent-navigation2 ros-eloquent-nav2-bringup

### Set work directory ###
WORKDIR /root
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
RUN source /opt/ros/eloquent/setup.bash
RUN mkdir -p ~/ros2_ws/src

RUN cd ~/ros2_ws/src \
&& git clone -b eloquent https://github.com/ros2/examples ros2_examples

RUN cd ~/ros2_ws \
&& wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos && vcs import src < turtlebot3.repos

RUN set -x
RUN echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

ADD ./tutrials/ ./ros2_ws/src/

RUN cd ~/ros2_ws/src \
&& ln -s ~/myProjects .

RUN cd ~/ros2_ws/src/utils \
&& rm -r hls_lfcd_lds_driver \
&& git clone -b eloquent-devel https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git

RUN cd ~/ros2_ws/src \
&& rm -r turtlebot3 \
&& git clone -b eloquent-devel https://github.com/ROBOTIS-GIT/turtlebot3.git \
&& git clone -b eloquent-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

RUN set -x
RUN cd ~/ros2_ws \
&& source /opt/ros/eloquent/setup.bash \
&& MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
