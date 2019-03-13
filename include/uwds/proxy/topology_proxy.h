#ifndef TOPOLOGY_PROXY_HPP
#define TOPOLOGY_PROXY_HPP

#include "proxy.h"
#include "uwds_msgs/GetTopology.h"
#include "uwds/types/topology.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds {

  class GetTopologyProxy : public DataProxy<Topology, GetTopology, bool>
  {
  public:
    GetTopologyProxy(NodeHandlePtr nh, ClientPtr client, TopologyPtr topology):DataProxy<Topology, GetTopology, bool>(nh, client, "uwds/get_topology", topology) {}

    ~GetTopologyProxy() = default;

    bool saveDataFromRemote(const GetTopology& get_topology_srv)
    {
      if (get_topology_srv.response.success)
      {
        this->data().reset(get_topology_srv.response.worlds, get_topology_srv.response.clients, get_topology_srv.response.client_interactions);
        return true;
      }
      return NULL;
    }
  protected:

    GetTopology fillRequest()
    {
      GetTopology get_topology_srv;
      return get_topology_srv;
    }

    string world_name_;
  };

  typedef boost::shared_ptr<GetTopologyProxy> GetTopologyProxyPtr;
  typedef boost::shared_ptr<GetTopologyProxy const> GetTopologyProxyConstPtr;

  class TopologyProxy
  {
  public:
    TopologyProxy(NodeHandlePtr nh, ClientPtr client)
    {
      topology_ = boost::make_shared<Topology>();
      get_topology_proxy_ = boost::make_shared<GetTopologyProxy>(nh, client, topology_);
    }

    ~TopologyProxy() {}

    bool getTopologyFromRemote()
    {
      return (get_topology_proxy_->getDataFromRemote());
    }

    Topology& topology() {return *topology_;}

  protected:

    TopologyPtr topology_;
    GetTopologyProxyPtr get_topology_proxy_;
  };

  typedef boost::shared_ptr<TopologyProxy> TopologyProxyPtr;
  typedef boost::shared_ptr<TopologyProxy const> TopologyProxyConstPtr;

}

#endif
