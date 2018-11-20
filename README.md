This ROS package contains the core components of **Underworlds**

## Introduction

**Underworlds** is a distributed and lightweight framework that aims at *sharing
between clients parallel models of the physical world surrounding a robot*.

The clients can be geometric reasoners (that compute topological relations between objects), motion planner, event monitors, viewers... any software that need to access a geometric (based on 3D meshes/voxels) and/or temporal (based on events) view of the world.

One of the main specific feature of **Underworlds** is the ability to store many parallel worlds: past models of the environment, future models, models with some objects filtered out, models that are physically consistent, etc.

This package provide the server that distribute and store the data and abstract classes for clients. So it is needed to develop your own clients (support **C++** and **Python**).

### Data structure

The **Underworlds** data structure is composed by a set of worlds, each world have his own timeline and scene tree. Meshes are centrally stored and distributed on-demand to the clients.

![data_structure](img/uwds_data_structure.png)

In a user point of view, **Underworlds** can be viewed as a dynamically reconfigurable reasoning pipeline of clients that read/write into worlds. Allowing to design and integrate the reasonners quickly.

### Installation instructions

Note : A Docker is available [here](https://github.com/underworlds-robot/uwds_dockerfile)

First install the dependencies with :
```
sudo apt-get install assimp-utils ros-kinetic-pose-cov-ops
sudo pip install pygraphviz uuid
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

### Introspection tools

Underworlds come with simple introspection tools. In order to visualize the clients topology, use the commands below :
```
rosrun uwds view_topology.py
evince topology.pdf
```

If you want to inspect the scene graph of a world use the command :
```
rosrun uwds view_scene.py [world]
evince scene.pdf
```

The data is also accessible from the ROS services :
* `uwds/get_scene`
* `uwds/get_timeline`
* `uwds/get_mesh`

### Record & replay

To record and replay with rosbag, record the topic `uwds/changes` :

```
rosbag record --lz4 -o example uwds/changes
```

### Build documentation
Use the following command :

`roscd uwds && rosdoc_lite . `

Then open the file `./doc/html/index.html` with your favorite application.
