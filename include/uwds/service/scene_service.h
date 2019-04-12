#ifndef SCENE_SERVICE_HPP
#define SCENE_SERVICE_HPP

#include "service.h"
#include "uwds/types/worlds.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds {
  class GetSceneService : public Service<GetScene::Request, GetScene::Response>
  {
  public:
    GetSceneService(NodeHandlePtr nh, ClientPtr client, WorldsPtr worlds):Service<GetScene::Request, GetScene::Response>(nh, client, "uwds/get_scene")
    {
      worlds_ = worlds;
    }
  protected:
    void fillResponse(GetScene::Request& req, GetScene::Response& res)
    {
      if(req.ctxt.world == "uwds")
      {
        res.success = false;
        res.error = "World namespace <uwds> reserved.";
        return;
      }
      if(req.ctxt.world == "")
      {
        res.success = false;
        res.error = "Empty world namespace.";
        return;
      }
      auto& scene = (*worlds_)[req.ctxt.world].scene();
      scene.lock();
      for (auto node : scene.nodes())
      {
        res.nodes.push_back(*node);
      }
      scene.unlock();
      res.root_id = scene.rootID();
      res.success = true;
    }

    WorldsPtr worlds_;
  };

  typedef boost::shared_ptr<GetSceneService> GetSceneServicePtr;
}

#endif
