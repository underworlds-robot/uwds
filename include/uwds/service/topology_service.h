#ifndef TOPOLOGY_SERVICE_HPP
#define TOPOLOGY_SERVICE_HPP

#include "service.h"
#include "uwds/types/topology.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds {
  class GetTopologyService : public Service<GetTopology::Request, GetTopology::Response>
  {
  public:
    GetTopologyService(NodeHandlePtr nh, ClientPtr client, TopologyPtr topology):Service<GetTopology::Request, GetTopology::Response>(nh, client, "uwds/get_topology")
    {
      topology_ = topology;
    }
  protected:
    void fillResponse(GetTopology::Request& req, GetTopology::Response& res)
    {
      topology_->lock();
      try
      {
        for (auto client : topology_->clients())
        {
          res.clients.push_back(*client);
        }
        for (auto client_interactions : topology_->clientsInteractions())
        {
          for(const auto client_interaction : *client_interactions)
          {
            if (std::find(res.worlds.begin(), res.worlds.end(), client_interaction->ctxt.world) == res.worlds.end())
               res.worlds.push_back(client_interaction->ctxt.world);
            res.client_interactions.push_back(*client_interaction);
          }
        }
        res.success = true;
      } catch(const std::exception& e) {
        res.success = false;
        res.error = e.what();
      }
      topology_->unlock();
    }

    TopologyPtr topology_;
  };

  typedef boost::shared_ptr<GetTopologyService> GetTopologyServicePtr;
}

#endif
