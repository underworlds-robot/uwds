# uwds
This ROS package contains the core components of Underworlds

(This package do not provide any clients)

### Installation instructions

A Docker for uwds is available [here](https://github.com/underworlds-robot/uwds_dockerfile)

#### Dependencies

Install the dependencies with :
```
sudo apt-get install assimp-utils ros-kinetic-pose-cov-ops
sudo pip install pygraphiz uuid
```
Then clone and build the catkin package :
```
cd catkin_ws/src
git clone https://github.com/underworlds-robot/uwds_msgs.git
git clone https://github.com/underworlds-robot/uwds.git
cd ..
catkin build uwds
source devel/setup.bash
```

### Launch instruction

To launch the Underworlds server use :
```
roslaunch uwds uwds_server.launch
```


### Build documentation
Use the following commands :

`roscd uwds && rosdoc_lite . `

Then open the file `./doc/html/index.html` with your favorite application.
