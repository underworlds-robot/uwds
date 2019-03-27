# Underworlds-ROS
This ROS package named ***uwds*** contains the core components of *Underworlds* as a **nodelet based architecture** that benefit from zero-copy pointer passing through topics.

Note : A Python Client API is provided in order to communicate with the server and develop your own clients, but will not benefit from this functionality.

----
<a href="http://www.youtube.com/watch?feature=player_embedded&v=sM5uTl-Klxo" target="blank"><p align="center"><img src="http://img.youtube.com/vi/sM5uTl-Klxo/0.jpg"
alt="IMAGE ALT TEXT HERE" width="480" height="360" border="10" /></p></a>

*Underworlds* is a distributed and lightweight framework that aims at **sharing between clients parallel models of the physical world surrounding a robot**.

The clients can be geometric reasoners (that compute topological relations between objects), motion planner, event monitors, viewers... any software that need to access a geometric (based on 3D meshes/voxels) and/or temporal (based on events) view of the world.

One of the main specific feature of *Underworlds* is the ability to **store many parallel worlds**: past models of the environment, future models, models with some objects filtered out, models that are physically consistent, etc.

This package provide the server that store the data and distribute it on-demand, abstract classes to develop your own clients and a basic set of common clients. To add more functionnalities, consider to install [this package](https://github.com/underworlds-robot/uwds_basic_clients) and [this one](https://github.com/underworlds-robot/uwds_physics_clients).

Follow the quick start guide to learn how to use *Underworlds* :

<a href="QUICKSTART.md"><p align="center">
  <img src="img/quick_start.png" alt="Quick start here !"/>
</p></a>

---
### References

[UNDERWORLDS : Cascading Situation Assessment for Robots](https://academia.skadge.org/publis/lemaignan2018underworlds.pdf)
