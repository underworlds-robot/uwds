#ifndef MESHES_SERVICE_HPP
#define MESHES_SERVICE_HPP

#include <ros/ros.h>
#include "uwds_msgs/Client.h"
#include <string>

using namespace std;
using namespace uwds_msgs;

namespace uwds {

  class GetMeshService : public Service<GetMesh::Request, GetMesh::Response>
  {
  public:
    GetMeshService(NodeHandlePtr nh, ClientPtr client, MeshesPtr meshes):Service<GetMesh::Request, GetMesh::Response>(nh, client, "uwds/get_mesh")
    {
      meshes_ = meshes;
    }

    void fillResponse(GetMesh::Request& req, GetMesh::Response& res)
    {
      if (meshes_->has(req.mesh_id))
      {
        res.mesh = (*meshes_)[req.mesh_id];
        res.success = true;
      } else {
        ROS_WARN("Mesh %s not existing",req.mesh_id.c_str());
        res.success = false;
        res.error = "Requested mesh <"+req.mesh_id+"> not existing";
      }
    }
  protected:
    MeshesPtr meshes_;
  };

  typedef boost::shared_ptr<GetMeshService> GetMeshServicePtr;


  class PushMeshService : public Service<PushMesh::Request, PushMesh::Response>
  {
  public:
    PushMeshService(NodeHandlePtr nh, ClientPtr client, MeshesPtr meshes):Service<PushMesh::Request, PushMesh::Response>(nh, client, "uwds/push_mesh")
    {
      meshes_ = meshes;
    }

    void fillResponse(PushMesh::Request& req, PushMesh::Response& res)
    {
      if (!meshes_->has(req.mesh.id))
      {
        meshes_->update(req.mesh);
        res.success = true;
      } else {
        ROS_WARN("Mesh <%s> already existing", req.mesh.id.c_str());
        res.success = false;
        res.error = "Pushed mesh <"+req.mesh.id+"> already existing";
      }
    }
  protected:
    MeshesPtr meshes_;
  };

  typedef boost::shared_ptr<PushMeshService> PushMeshServicePtr;

}

#endif
