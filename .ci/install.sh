#!/bin/bash

set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

MY_PATH=`pwd`

echo "Starting install preparation"

sudo apt-get -y install git

echo "clone uav_core"
cd
git clone https://github.com/ctu-mrs/uav_core.git
cd uav_core

echo "running the main install.sh"
./installation/install.sh

cd $MY_PATH
./install_dependencies.sh

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/uav_core
ln -s "$MY_PATH" mtsp_planning_task
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws

echo "installation part ended"
