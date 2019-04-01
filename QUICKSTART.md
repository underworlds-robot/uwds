# Underworlds Quick Start
## Introduction

The aim of this quick start guide is to give a basic understanding of what is Underworlds, it’s purpose and it’s applications through basic examples in C++ and Python.  During this guide, we assume that you have a basic understanding of ROS, if it is not the case, follow the [ROS tutorials](https://wiki.ros.org/ROS/Tutorials).

### Core concept

In a classic robotic architecture, the situation assessment component is in charge of generating a set of descriptions in order to give to the robot a comprehension of it's surrounding environment. In the context of human-robot collaborations, the geometry and the relations between agents or objects are particularly important, they allow to ground verbal expressions, perform collaborative task and motion planning or supervise the correct execution of a shared plan.

Underworlds is a framework that focuses on maintaining and distributing multiple/alternative geometric (based on 3D bounding boxes and/or 3D meshes) and symbolic models of the physical world (based on events) over loosely coupled clients that provide reasoning capabilities.

<p align="center">
  <img src="img/uwds_architecture.png" alt="uwds_architecture"/>
</p>

This architecture allow developers to design high level reasoning in the same modular way than perception pipelines. Allowing to combine dynamically situation-assessment components depending on the task and to implement quickly new geometric and symbolic reasoners.

It can be viewed as a set of distributed world states that share the same data-structure : A scene graph (that is actually a tree) where each node referent his parent with a relative position, velocity and acceleration (with they respective covariances); a timeline of situations (event or temporal predicates that can represent actions or facts for example) and the meshes that are centrally stored and served on-demand.

### Documentation

Documentations can be found here :
* [API Documentation]()
* [MSG Documentation]()

## Installation instructions (tested on ROS Kinetic)

First install the dependencies with :
```
sudo apt-get install assimp-utils ros-$ROS_DISTRO-pose-cov-ops ros-$ROS_DISTRO-jsk-recognition-msgs ros-$ROS_DISTRO-jsk-visualization-msgs ros-$ROS_DISTRO-jsk-visualization
sudo pip install pygraphviz uuid
```
Then clone and build the catkin package :
```
cd catkin_ws/src
git clone https://github.com/underworlds-robot/uwds_msgs.git
git clone https://github.com/underworlds-robot/uwds.git
cd ..
catkin_make
source devel/setup.bash
```

Note : A Docker is available [here](https://github.com/underworlds-robot/uwds_dockerfile)

## First launch

First, launch the server with :
```
roslaunch uwds uwds_server.launch
```
Launch the *env_provider* that will create a world from a 3D file :
```
roslaunch uwds env_provider.launch
```
And after that launch the scene viewer that will publish the visualization topics :

```
roslaunch uwds scene_viewer.launch
```

Then start Rviz with :
```
rviz
```
Verify that you have set the global frame of Rviz to `map`. Then add two types of Display, a `BoundingBoxArray`(available in the jsk_visualization package) and a `MarkerArray` display.

Choose the only topic available in each display (it should be respectively `robot/env/boxes` and `robot/env/meshes`).

When you are done, you should have something like this :

<p align="center">
  <img src="img/example.png" alt="example"/>
</p>

## First introspection

Underworlds comes with two simple introspection tool that are usefull for debugging purposes !

In order to visualize the clients topology(e.g. the reasoning pipeline), use the following commands :
```
rosrun uwds view_topology.py
evince topology.pdf
```

You should have something like that :

<p align="center">
  <img src="img/topology.png" alt="topology"/>
</p>

This figure means that the *env_provider* writes into the world named *env* and the *scene_viewer* reads it.

You can also inspect the scene graph of the visualized world by using :
```
rosrun uwds view_scene.py env
evince scene.pdf
```

This should generate this :

<p align="center">
  <img src="img/scene.png" alt="scene"/>
</p>

As you notice, every node is at least parented to the root of the tree. The scene can be viewed in the same manner that the traditional `tf`.

# Build your own clients

## C++ Minimal working examples

In this section you will learn how to make your own clients by using the C++ Client API. The examples given are simple in order to focus on the core concepts. To know how to program optimized Underworlds clients, see the last section !


#### Provider example

In this first example we will make a client that load an Assimp compatible 3D file into an Underworlds world :
```c++
#include <ros/ros.h>
#include <uwds/uwds.h>

typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
using namespace std;
using namespace std_msgs;
using namespace uwds;

class ProviderExample
{
public:
  ProviderExample(NodeHandlePtr nh, NodeHandlePtr pnh, string filename)
  {
    ctx_ = boost::make_shared<UnderworldsProxy>(nh, pnh, "provider_example", PROVIDER);
    if(ctx_->worlds()["robot/env"].pushSceneFrom3DFile(filename))
      ROS_INFO("Successfully load file !");
  }
protected:
  UnderworldsProxyPtr ctx_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "provider_example");
  NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  NodeHandlePtr pnh = boost::make_shared<ros::NodeHandle>("~");
  string filename;
  nh->param<string>("filename", filename, "");
  ProviderExample provider = ProviderExample(nh, pnh, filename);
  ros::spin();
}

```

Let's see in details the code :

```c++
#include <ros/ros.h>
#include <uwds/uwds.h>

typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
using namespace std;
using namespace std_msgs;
using namespace uwds;
```

Here we include ROS and Underworlds (aka *uwds*), and we declare some convenient stuff. Then we declare a component that have a smart pointer to an Underworlds proxy.

```c++
class ProviderExample
{
public:
  ProviderExample(NodeHandlePtr nh, NodeHandlePtr pnh, string filename)
  {
    // [...]
  }
protected:
  UnderworldsProxyPtr ctx_;
};
```
Here is the first important thing :
```c++
ctx_ = boost::make_shared<UnderworldsProxy>(nh, pnh, "provider_example", PROVIDER);

```
This line create the Underworlds proxy and declare the client as a `PROVIDER` that is named *provider_example*, the proxy is the access point to the Underworlds data-structure and allow you to not care about the communication processes that handle the distribution of the world states to the clients. You just need to create it to use Underworlds, here we use shared pointers but it is not required.

The second important line in this example is this one :

```c++
ctx_->worlds()["robot/env"].pushSceneFrom3DFile(filename)
```
This line will lazzely create a proxy for the world `robot/env` that will fetch the world from the Underworlds server to then subscribe to any changes that will pass trought the topic `robot/env/changes`. After that the proxy will load the scene from the given file and send it over the network.

#### Reader example

```c++
#include <ros/ros.h>
#include <uwds/uwds.h>

typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
using namespace std;
using namespace std_msgs;
using namespace uwds;

class ReaderExample
{
public:
  ReaderExample(NodeHandlePtr nh, NodeHandlePtr pnh)
  {
    ctx_ = boost::make_shared<UnderworldsProxy>(nh, pnh, "reader_example", READER);
    if(ctx_->worlds()["robot/env_filtered"].connect(bind(&ReaderExample::onChanges, this, _1, _2, _3)));
      ROS_INFO("Ready to listen for changes !");
  }
protected:
  void onChanges(string world_name, Header header, Invalidations invalidations)
  {
    for(const auto id : invalidations.node_ids_updated)
      ROS_INFO("Received node <(%s)%s>", ctx_->worlds()["robot/env_filtered"].scene().nodes()[id].name.c_str(), id.c_str());
    for(const auto id : invalidations.situation_ids_updated)
      ROS_INFO("Received situation <(%s)%s>", ctx_->worlds()["robot/env_filtered"].timeline().situations()[id].description.c_str(), id.c_str());
    for(const auto id : invalidations.mesh_ids_updated)
      ROS_INFO("Received mesh <%s>", ctx_->meshes()[id].id.c_str());
  }
  UnderworldsProxyPtr ctx_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reader_example");
  NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  NodeHandlePtr pnh = boost::make_shared<ros::NodeHandle>("~");
  ReaderExample reader = ReaderExample(nh, pnh);
  ros::spin();
}

```

#### Filter example

```c++
#include <ros/ros.h>
#include <uwds/uwds.h>

typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
using namespace std;
using namespace std_msgs;
using namespace uwds;

class FilterExample
{
public:
  FilterExample(NodeHandlePtr nh, NodeHandlePtr pnh, string regex)
  {
    regex_ = std::regex(regex);
    ctx_ = boost::make_shared<UnderworldsProxy>(nh, pnh, "filter_example", FILTER);
    if(ctx_->worlds()["robot/env"].connect(bind(&FilterExample::onChanges, this, _1, _2, _3)));
  }
protected:
  void onChanges(string world_name, Header header, Invalidations invalidations)
  {
    Changes changes;
    auto& scene = ctx_->worlds()["robot/env"].scene();
    for(const auto& node_id : invalidations.node_ids_updated)
    {
      if(regex_match(scene.nodes()[node_id].name, regex_))
      {
        changes.nodes_to_update.push_back(scene.nodes()[node_id]);
      }
    }
    ctx_->worlds()["robot/env_filtered"].update(changes);
  }
  regex regex_;
  string input_world_;
  string output_world_;
  UnderworldsProxyPtr ctx_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter_example");
  NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  NodeHandlePtr pnh = boost::make_shared<ros::NodeHandle>("~");
  string regex;
  nh->param<string>("regex", regex, "");
  FilterExample filter = FilterExample(nh, pnh, regex);
  ros::spin();
}

```

### Python Minimal working examples

#### Provider example

#### Filter example

#### Reader example

## Good practices and optimization

### Launch files

### Nodelet-based architecture
