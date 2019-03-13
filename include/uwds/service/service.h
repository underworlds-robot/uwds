#ifndef SERVICE_HPP
#define SERVICE_HPP

#include <ros/ros.h>
#include "uwds_msgs/Client.h"
#include <string>

using namespace std;
using namespace uwds_msgs;

namespace uwds {

  typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
  typedef boost::shared_ptr<ros::ServiceServer> ServiceServerPtr;

  template<typename ServiceMessage>
  class Service
  {
  public:
    Service(NodeHandlePtr nh, ClientPtr client, string service_name)
    {
      nh_ = nh;
      client_ = client;
      service_name_ = service_name;
      service_server_ = boost::make_shared<ros::ServiceServer>(nh_->advertiseService(service_name_,
                                              &Service::onRequest,
                                              this));
    }

    ~Service() {}

  protected:

    bool onRequest(ServiceMessage srv_msg)
    {
      try {
        fillResponse(srv_msg);
      } catch (std::exception e) {
        ROS_ERROR("[%s::serviceServer] Exception occured when processing '%s' : %s", client_->name.c_str(), service_name_.c_str(), e.what());
      }
      return true;
    }

    virtual void fillResponse(const Service& srv_msg) = 0;

    NodeHandlePtr nh_;

    ClientPtr client_;

    string service_name_;

    ServiceServerPtr service_server_;
  };
}

#endif
