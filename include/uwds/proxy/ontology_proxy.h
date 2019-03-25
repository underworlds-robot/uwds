#ifndef ONTOLOGY_PROXY_HPP
#define ONTOLOGY_PROXY_HPP

#include "proxy.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds {

  class QueryOntologyProxy : public ServiceProxy<QueryOntology, string>
  {
  public:
    QueryOntologyProxy(NodeHandlePtr nh, NodeHandlePtr pnh, ClientPtr client, string world_name):ServiceProxy<QueryOntology, string>(nh, pnh, client, "uwds/query_ontology")
    {
      world_name_ = world_name;
    }

  protected:
    QueryOntology fillRequest(string query)
    {
      QueryOntology query_srv;
      query_srv.request.ctxt.client = *client_;
      query_srv.request.ctxt.world = world_name_;
      query_srv.request.query = query;
      return query_srv;
    }

    string world_name_;
  };

  typedef boost::shared_ptr<QueryOntologyProxy> QueryOntologyProxyPtr;
  typedef boost::shared_ptr<QueryOntologyProxy const> QueryOntologyProxyConstPtr;

  class OntologyProxy
  {
  public:
    OntologyProxy(NodeHandlePtr nh, NodeHandlePtr pnh, ClientPtr client, string world_name)
    {
      query_ontology_proxy_ = boost::make_shared<QueryOntologyProxy>(nh, pnh, client, world_name);
    }

    vector<string> queryOntology(string query)
    {
      QueryOntology query_srv = query_ontology_proxy_->call(query);
      if(!query_srv.response.success)
        ROS_ERROR("[%s::ontology] Exception occured when processing '%s' query", query_ontology_proxy_->client().name.c_str(), query.c_str());
      return query_srv.response.result;
    }
  protected:
    QueryOntologyProxyPtr query_ontology_proxy_;
  };

  typedef boost::shared_ptr<OntologyProxy> OntologyProxyPtr;
  typedef boost::shared_ptr<OntologyProxy const> OntologyProxyConstPtr;

}

#endif
