This ROS package contains the core components of **Underworlds**

## Introduction

**Underworlds** is a distributed and lightweight framework that aims at *sharing
between clients parallel models of the physical world surrounding a robot*.

The clients can be geometric reasoners (that compute topological relations between objects), motion planner, event monitors, viewers... any software that need to access a geometric (based on 3D meshes/voxels) and/or temporal (based on events) view of the world.

One of the main specific feature of **Underworlds** is the ability to store many parallel worlds: past models of the environment, future models, models with some objects filtered out, models that are physically consistent, etc.

This package provide the server that distribute and store the data and abstract classes for clients. So it is needed to develop your own clients (support **C++** and **Python**).

### Data structure

![data_structure](img/uwds_data_struture.png)

### Nodelets

##### UwdsServerNodelet

This nodelet allow to centralize and broadcast the changes while serving Underworlds data structures to the clients for initialization.

###### Parameters
 - `~clean_up_timer_duration` duration of the clean up timer (default : 5)
 - `~situations_buffer_size` duration of the situations buffer (default : 300)
 - `~use_multithread_callback` if true use multithread callback (default : true)
 - `~publisher_buffer_size` the publishers buffer size (default : 10)
 - `~subscriber_buffer_size` the subscribers buffer size (default : 10)
 - `~time_synchronizer_buffer_size` the time sync buffer size (default : 10)

###### Subscribed topics
- `uwds/changes` allow to receive the changes from clients

###### Publisher topics
- publish to the `[world]/changes` of the worlds

###### Advertised services
- `uwds/get_topology` allow to fetch the clients topology
- `uwds/get_scene` allow to fetch the scene
- `uwds/get_mesh` allow to fetch the meshes
- `~input_worlds` list the input worlds
- `~output_world` list the output worlds

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
