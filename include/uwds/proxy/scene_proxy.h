#ifndef SCENE_PROXY_HPP
#define SCENE_PROXY_HPP

#include "proxy.h"
#include "meshes_proxy.h"
#include "uwds_msgs/GetScene.h"
#include "uwds/types/scene.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds {

  class GetSceneProxy : public DataProxy<Scene, GetScene>
  {
  public:
    GetSceneProxy(NodeHandlePtr nh, NodeHandlePtr pnh, ClientPtr client, string world_name, ScenePtr scene, MeshesProxyPtr meshes):DataProxy<Scene, GetScene>(nh, pnh, client, "uwds/get_scene", scene)
    {
      world_name_ = world_name;
      meshes_ = meshes;
    }

    ~GetSceneProxy() = default;

    Invalidations saveDataFromRemote(const GetScene& get_scene_srv)
    {
      Invalidations invalidations;
      if (get_scene_srv.response.success)
      {
        invalidations.node_ids_deleted = this->data().reset(get_scene_srv.response.root_id);
        for(auto& node : get_scene_srv.response.nodes)
        {
          std::vector<std::string> mesh_ids;
          for(const auto& property : node.properties)
    			{
            if(property.name == "meshes" && !property.data.empty())
    				{
              boost::split(mesh_ids, property.data, boost::is_any_of(","), boost::token_compress_on);
              for (const auto mesh_id : mesh_ids)
    					{
    						if(meshes_->getMeshFromRemote(mesh_id))
    							invalidations.mesh_ids_updated.push_back(mesh_id);
    					}
            }
          }
          invalidations.node_ids_updated.push_back(data_->update(node));
        }
      }
      return invalidations;
    }

  protected:
    GetScene fillRequest()
    {
      GetScene get_scene_srv;
      get_scene_srv.request.ctxt.client = *client_;
      get_scene_srv.request.ctxt.world = world_name_;
      return get_scene_srv;
    }

    string world_name_;

    MeshesProxyPtr meshes_;
  };

  typedef boost::shared_ptr<GetSceneProxy> GetSceneyProxyPtr;
  typedef boost::shared_ptr<GetSceneProxy const> GetSceneProxyConstPtr;

  class SceneProxy
  {
  public:
    SceneProxy(NodeHandlePtr nh, NodeHandlePtr pnh, ClientPtr client, string world_name, MeshesProxyPtr meshes_proxy)
    {
      scene_ = boost::make_shared<Scene>();
      get_scene_proxy_ = boost::make_shared<GetSceneProxy>(nh, pnh, client, world_name, scene_, meshes_proxy);
    }

    ~SceneProxy() {}

    Invalidations getSceneFromRemote()
    {
      return get_scene_proxy_->getDataFromRemote();
    }

    Scene& scene() {return *scene_;}

  protected:
    ScenePtr scene_;
    GetSceneyProxyPtr get_scene_proxy_;
  };

  typedef boost::shared_ptr<SceneProxy> SceneProxyPtr;
  typedef boost::shared_ptr<SceneProxy const> SceneProxyConstPtr;

}

#endif
