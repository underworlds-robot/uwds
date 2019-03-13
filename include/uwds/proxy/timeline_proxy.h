#ifndef TIMELINE_PROXY_HPP
#define TIMELINE_PROXY_HPP

#include "proxy.h"
#include "uwds_msgs/GetTimeline.h"
#include "uwds/types/timeline.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds {

  class GetTimelineProxy : public DataProxy<Timeline, GetTimeline>
  {
  public:
    GetTimelineProxy(NodeHandlePtr nh, ClientPtr client, string world_name, TimelinePtr timeline):DataProxy<Timeline, GetTimeline>(nh, client, "uwds/get_timeline", timeline)
    {
      world_name_ = world_name;
    }

    ~GetTimelineProxy() = default;

    Invalidations saveDataFromRemote(const GetTimeline& get_timeline_srv)
    {
      Invalidations invalidations;
      if (get_timeline_srv.response.success)
      {
        invalidations.situation_ids_deleted = data_->reset(get_timeline_srv.response.origin.data);
        invalidations.situation_ids_updated = data_->update(get_timeline_srv.response.situations);
      } else {
        std::runtime_error(get_timeline_srv.response.error);
      }
      return invalidations;
    }

  protected:

    GetTimeline fillRequest()
    {
      GetTimeline get_timeline_srv;
      get_timeline_srv.request.ctxt.client = *client_;
      get_timeline_srv.request.ctxt.world = world_name_;
      return get_timeline_srv;
    }

    string world_name_;
  };

  typedef boost::shared_ptr<GetTimelineProxy> GetTimelineProxyPtr;
  typedef boost::shared_ptr<GetTimelineProxy const> GetTimelineProxyConstPtr;

  class TimelineProxy
  {
  public:
    TimelineProxy(boost::shared_ptr<ros::NodeHandle> nh, ClientPtr client, string world_name)
    {
      timeline_ = boost::make_shared<Timeline>();
      get_timeline_proxy_ = boost::make_shared<GetTimelineProxy>(nh, client, world_name, timeline_);
    }

    ~TimelineProxy() {}

    Invalidations getTimelineFromRemote()
    {
      return get_timeline_proxy_->getDataFromRemote();
    }

    Timeline& timeline() {return *timeline_;}

  protected:
    TimelinePtr timeline_;
    GetTimelineProxyPtr get_timeline_proxy_;
  };

  typedef boost::shared_ptr<TimelineProxy> TimelineProxyPtr;
  typedef boost::shared_ptr<TimelineProxy const> TimelineProxyConstPtr;

}

#endif
