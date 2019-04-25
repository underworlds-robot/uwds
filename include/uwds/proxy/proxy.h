#ifndef PROXY_HPP
#define PROXY_HPP

#include <ros/ros.h>
#include "uwds_msgs/Client.h"
#include <string>

using namespace std;
using namespace uwds_msgs;

namespace uwds {
  // for covenience
  typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
  typedef boost::shared_ptr<ros::ServiceClient> ServiceClientPtr;

  /** @brief
   * This class ease the creation of ROS based proxies.
   */
  template<typename ServiceMessage, typename... RequestParameters>
  class ServiceProxy
  {
    typedef boost::shared_ptr<ros::ServiceClient> ServiceClientPtr;

  public:
    /** @brief
     * The constructor.
     * @param nh The ROS NodeHandle shared ptr
     * @param pnh The private ROS nodehandle shared ptr
     * @param client The client shared ptr
     * @param service_name The service name
     */
    ServiceProxy(NodeHandlePtr nh, NodeHandlePtr pnh, ClientPtr client, string service_name)
    {
      nh_ = nh;
      pnh_ = pnh;
      client_ = client;
      service_name_ = service_name;
      service_client_ = boost::make_shared<ros::ServiceClient>(nh_->serviceClient<ServiceMessage>(service_name_, false));
    }

    /** @brief
     * The destructor.
     */
    ~ServiceProxy() {}

    /** @brief
     * The client accessor.
     */
    Client& client() {return *client_;}

    /** @brief
     * The service name accessor.
     */
    string serviceName() {return service_name_;}

    /** @brief
     * The call service method.
     * @param parameters The request parameters
     */
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
    /** @brief
     * This method fill the request message.
     * This method need to be implemented in the sub-classes
     * @param parameters The request parameters
     */
    virtual ServiceMessage fillRequest(RequestParameters... parameters) = 0;

    /** @brief
     * The ROS NodeHandle shared ptr.
     */
    NodeHandlePtr nh_;

    /** @brief
     * The private ROS NodeHandle shared ptr.
     */
    NodeHandlePtr pnh_;

    /** @brief
     * The service name.
     */
    string service_name_;

    /** @brief
     * The client shared ptr.
     */
    ClientPtr client_;

    /** @brief
     * The ROS service client shared ptr.
     */
    ServiceClientPtr service_client_;
  };

  /** @brief
   * This class ease the creation of ROS based proxies that manage local data-structure.
   */
  template<class Data, class GetDataSrv, class Returns = Invalidations, typename... Parameters>
  class DataProxy : public ServiceProxy<GetDataSrv, Parameters... >
  {
    typedef boost::shared_ptr<Data> DataPtr;

  public:
    /** @brief
     * The constructor.
     */
    DataProxy(NodeHandlePtr nh, NodeHandlePtr pnh, ClientPtr client, string service_name, boost::shared_ptr<Data> data):ServiceProxy<GetDataSrv, Parameters...>(nh, pnh, client, service_name)
    {
      data_ = data;
    }

    /** @brief
     * The destructor.
     */
    ~DataProxy() {}

    /** @brief
     * The data accessor.
     */
    Data& data() {return *data_;}

    /** @brief
     * The method to fetch data from remote.
     */
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
    /** @brief
     * This method fill the request message.
     * This method need to be implemented in the sub-classes
     * @param parameters The request parameters
     */
    virtual GetDataSrv fillRequest(Parameters... parameters) = 0;

    /** @brief
     * This method save the data requested.
     * This method need to be implemented in the sub-classes
     * @param parameters The request parameters
     */
    virtual Returns saveDataFromRemote(const GetDataSrv& srv) = 0;

    /** @brief
     * The data shared ptr.
     */
    DataPtr data_;
  };


}

#endif
