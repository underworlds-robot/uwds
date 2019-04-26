#!/bin/bash
echo "Start downloding dependencies... this may take a while..."
sudo apt-get install --assume-yes assimp-utils ros-$ROS_DISTRO-pose-cov-ops ros-$ROS_DISTRO-jsk-recognition-msgs ros-$ROS_DISTRO-jsk-visualization-msgs ros-$ROS_DISTRO-jsk-visualization ros-$ROS_DISTRO-ar-track-alvar ros-$ROS_DISTRO-octomap ros-$ROS_DISTRO-doc-lite
pip install -r requirements.txt
cd ..
git clone https://github.com/underworlds-robot/uwds_msgs.git
cd -
cd ~
git clone https://github.com/bulletphysics/bullet3.git
python ./bullet3/setup.py install --prefix=${HOME}
cd -
echo "Bye bye !"
