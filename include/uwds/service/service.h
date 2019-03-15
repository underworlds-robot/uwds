#ifndef SERVICE_HPP
#define SERVICE_HPP

#include <ros/ros.h>
#include "uwds_msgs/Client.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds {

  typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
  typedef boost::shared_ptr<ros::ServiceServer> ServiceServerPtr;

  template<typename RequestMessage, typename ResponseMessage>
  class Service
  {
  public:
    Service(NodeHandlePtr nh, ClientPtr client, string service_name)
    {
      nh_ = nh;
      client_ = client;
      service_name_ = service_name;
      service_server_ = nh_->advertiseService(service_name_,
                                              &Service::onRequest,
                                              this);
    }

    ~Service() {}

  protected:

    bool onRequest(RequestMessage& req, ResponseMessage& res)
    {
      try {
        fillResponse(req, res);
      } catch (std::exception e) {
        ROS_ERROR("[%s::serviceServer] Exception occured when processing '%s' : %s", client_->name.c_str(), service_name_.c_str(), e.what());
      }
      return true;
    }

    virtual void fillResponse(RequestMessage& req, ResponseMessage& res) = 0;

    NodeHandlePtr nh_;

    ClientPtr client_;

    string service_name_;

    ros::ServiceServer service_server_;
  };
}

#endif
