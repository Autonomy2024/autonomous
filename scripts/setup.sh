#!/bin/bash

UPDATE_ROS_NOITIC="true"

# check ubuntu version
# otherwise warn and point to docker?
UBUNTU_RELEASE="`lsb_release -rs`"

if [[ "${UBUNTU_RELEASE}" == "14.04" ]]; then
	echo "Ubuntu 14.04 is no longer supported"
	exit 1
elif [[ "${UBUNTU_RELEASE}" == "16.04" ]]; then
	echo "Ubuntu 16.04 is no longer supported"
	exit 1
elif [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
	echo "Ubuntu 18.04"
elif [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then
	echo "Ubuntu 20.04"
elif [[ "${UBUNTU_RELEASE}" == "22.04" ]]; then
	echo "Ubuntu 22.04"
    exit 1
fi

if [ "$UPDATE_ROS_NOITIC" != "false" ] then
    echo "start update ros noetic"
    sudo apt-get update && sudo apt-get upgrade
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
fi

# Flight Controller update

PX4_FIRMWARE=${HOME}/PX4_Firmware
if [ ! -d "$PX4_FIRMWARE" ]; then
    cd
    cp ~/PX4_Firmware.zip ~/ -rf
    unzip PX4_Firmware.zip
    sync
    pip install numpy==1.24.0
    ~/PX4_FIRMWARE/Tools/setup/ubuntu.sh
fi

sudo apt install xmlstarlet

CATKIN_WS=${HOME}/catkin_ws
CATKIN_SRC=${HOME}/catkin_ws/src
AUTONOMOUS=${HOME}/catkin_ws/src/autonomous

if [ ! -d "$CATKIN_WS" ]; then
    echo "Creating $CATKIN_WS ... "
    mkdir -p $CATKIN_SRC
fi

if [ ! -d "$CATKIN_SRC" ]; then
    echo "Creating $CATKIN_SRC ..."
fi

if [ ! -d "$AUTONOMOUS" ]; then
    echo "copy $autonomous ..."
    cp ~/env/autonomous.zip ${HOME}/catkin_ws/src
    cd ${HOME}/catkin_ws/src
    unzip autonomous.zip
fi

# Install libgeographic and geographiclib
sudo apt-get install libgeographic-dev
sudo apt-get install geographiclib-tools
sudo apt-get install ros-noetic-mavlink
sudo apt-get install ros-noetic-geographic-msgs

# install geographiclib datasets
cd ~/env
unzip GeographicLib.zip
sudo cp ~/env/GeographicLib /usr/share -rf
sudo ldconfig

# Update some gazebo models
cp ~/env/models.zip ~/.gazebo/
cd ~/.gazebo/models/
unzip models.zip

# VIO tools update
sudo apt install libgoogle-glog-dev
sudo apt-get install libsuitesparse-dev
cd ~/env/
unzip ceres-solver-1.14.0.zip
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig

# compile autonomous
sudo apt install python3-pip python3-rosdep python3-catkin-pkg python3-rospkg
sudo apt install python3-pip
sudo pip3 install catkin-tools
cd ~/catkin_ws/
catkin_make

# Add some setting to .bashrc
echo "~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware" >> ~/.bashrc
echo "ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo" >> ~/.bashrc
source ~/.bashrc

echo "Thanks to wait, now update finished"