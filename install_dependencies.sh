#!/bin/bash
echo "Start downloding dependencies... this may take a while..."
sudo apt-get install --assume-yes libbullet-dev assimp-utils ros-$ROS_DISTRO-pose-cov-ops ros-$ROS_DISTRO-jsk-recognition-msgs ros-$ROS_DISTRO-jsk-visualization ros-$ROS_DISTRO-ar-track-alvar ros-$ROS_DISTRO-octomap ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-mapping ros-$ROS_DISTRO-rosdoc-lite
pip install -r requirements.txt --user
cd ..
git clone https://github.com/underworlds-robot/uwds_msgs.git
cd -
echo "Bye bye !"
