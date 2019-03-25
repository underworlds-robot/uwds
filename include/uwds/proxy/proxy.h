#ifndef PROXY_HPP
#define PROXY_HPP

#include <ros/ros.h>
#include "uwds_msgs/Client.h"
#include <string>

using namespace std;
using namespace uwds_msgs;

namespace uwds {

  typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
  typedef boost::shared_ptr<ros::ServiceClient> ServiceClientPtr;

  template<typename ServiceMessage, typename... RequestParameters>
  class ServiceProxy
  {
    typedef boost::shared_ptr<ros::ServiceClient> ServiceClientPtr;

  public:
    ServiceProxy(NodeHandlePtr nh, NodeHandlePtr pnh, ClientPtr client, string service_name)
    {
      nh_ = nh;
      pnh_ = pnh;
      client_ = client;
      service_name_ = service_name;
      service_client_ = boost::make_shared<ros::ServiceClient>(nh_->serviceClient<ServiceMessage>(service_name_));
    }

    ~ServiceProxy() {}

    Client& client() {return *client_;}

    string serviceName() {return service_name_;}

    virtual ServiceMessage call(RequestParameters... parameters)
    {
      if(!service_client_->exists())
      {
        ROS_WARN("[%s::serviceProxy] Waiting for '%s' service...", client_->name.c_str(), service_name_.c_str());
        service_client_->waitForExistence();
      }
      ServiceMessage service = fillRequest(parameters...);
      if (!service_client_->call(service))
        ROS_ERROR("[%s::serviceProxy] Error occurred while calling '%s' service", client_->name.c_str(), service_name_.c_str());
      return service;
    }

  protected:
    virtual ServiceMessage fillRequest(RequestParameters... parameters) = 0;

    NodeHandlePtr nh_;

    NodeHandlePtr pnh_;

    string service_name_;

    ClientPtr client_;

    ServiceClientPtr service_client_;
  };


  template<class Data, class GetDataSrv, class Returns = Invalidations, typename... Parameters>
  class DataProxy : public ServiceProxy<GetDataSrv, Parameters... >
  {
    typedef boost::shared_ptr<Data> DataPtr;

  public:

    DataProxy(NodeHandlePtr nh, NodeHandlePtr pnh, ClientPtr client, string service_name, boost::shared_ptr<Data> data):ServiceProxy<GetDataSrv, Parameters...>(nh, pnh, client, service_name)
    {
      data_ = data;
    }

    ~DataProxy() {}

    Data& data() {return *data_;}

    virtual Returns getDataFromRemote(Parameters... parameters)
    {
      Returns returns;
      try {
        returns = saveDataFromRemote(this->call(parameters...));
      } catch (std::exception e) {
        ROS_ERROR("[%s::dataProxy] Error occured when saving '%s' data into local data-structure : %s", this->client().name.c_str(), this->serviceName().c_str(), e.what());
      }
      return returns;
    }

  protected:

    virtual GetDataSrv fillRequest(Parameters... parameters) = 0;

    virtual Returns saveDataFromRemote(const GetDataSrv& srv) = 0;

    DataPtr data_;
  };


}

#endif
