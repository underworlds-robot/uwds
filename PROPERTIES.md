# Underworlds properties
In order to be extendable Underworlds store the optional informations of the nodes/situations/meshes into a general purpose properties map. The aim of this table is to describe the properties used in this package.

*Note : You can add your own properties as you handle them in your clients*

### Nodes properties

|Property name| Property data format | NodeType | description |
|---|---|---|---|
| aabb\* | x_dim,y_dim,z_dim | MESH | The AABB of the node |
| meshes  | mesh_id,mesh_id,...,mesh_id | MESH  | The list of the meshes UUID attached to this node |
| octrees  | octree_id,octree_id,...,octree_id | MESH  | The list of the octrees UUID attached to this node |
| hfov\*  | hfov | CAMERA | The horizontal field of view of the camera |
| aspect\*  | aspect  | CAMERA | The aspect ratio of the camera |
| up  | x_dim,y_dim_,z_dim | CAMERA | The up vector of the camera |

\* required property

### Situations properties

|Property name | Property data format | SituationType | description |
|---|---|---|---|
| subject\* | subject_id | all | The subject UUID of the situation |
| object | object_id | all | The object UUID of the situation|
| predicate | predicate | FACT | The predicate that match the knowledge model |
| action | action | ACTION | The action that match the action model|

\* required property
### Meshes properties

|Property name | Property data format | SituationType | description |
|---|---|---|---|
| diffuse | | | |
| specular | | | |

### Octree properties

TODO
