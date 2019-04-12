#ifndef TIMELINE_SERVICE_HPP
#define TIMELINE_SERVICE_HPP

#include "service.h"
#include "uwds/types/worlds.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds {
  class GetTimelineService : public Service<GetTimeline::Request, GetTimeline::Response>
  {
  public:
    GetTimelineService(NodeHandlePtr nh, ClientPtr client, WorldsPtr worlds):Service<GetTimeline::Request, GetTimeline::Response>(nh, client, "uwds/get_timeline")
    {
      worlds_ = worlds;
    }
  protected:
    void fillResponse(GetTimeline::Request& req, GetTimeline::Response& res)
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
      auto& timeline = (*worlds_)[req.ctxt.world].timeline();
      timeline.lock();
      for (const auto& situation : timeline.situations())
      {
        res.situations.push_back(*situation);
      }
      timeline.unlock();
      res.origin = timeline.origin();
      res.success = true;
    }

    WorldsPtr worlds_;
  };

  typedef boost::shared_ptr<GetTimelineService> GetTimelineServicePtr;
}

#endif
