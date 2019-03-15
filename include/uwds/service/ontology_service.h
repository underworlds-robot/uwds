#ifndef ONTOLOGY_SERVICE_HPP
#define ONTOLOGY_SERVICE_HPP

#include "service.h"
#include "uwds/proxy/worlds_proxy.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds {

  class Ontology : public enable_shared_from_this<Ontology>
  {
  public:
    Ontology(NodeHandlePtr nh)
    {
      nh_ = nh;
    }
    virtual bool updateIndividual(string world_name, Node node) = 0;
    virtual bool removeIndividual(string world_name, string node_id) = 0;
    virtual bool updateSituation(string world_name, Situation situation) = 0;
    virtual bool removeSituation(string world_name, string situation_id) = 0;
    virtual vector<string> queryOntology(string world_name, string query) = 0;
  protected:
    NodeHandlePtr nh_;
  };

  typedef boost::shared_ptr<Ontology> OntologyPtr;
  typedef boost::weak_ptr<Ontology> OntologyWeakPtr;
  typedef boost::shared_ptr<Ontology const> OntologyConstPtr;

  class QueryOntologyService : public Service<QueryOntology::Request, QueryOntology::Response>
  {
  public:
    QueryOntologyService(NodeHandlePtr nh, ClientPtr client, WorldsProxyPtr worlds, OntologyPtr ontology):Service<QueryOntology::Request, QueryOntology::Response>(nh, client, "uwds/query_ontology")
    {
      worlds_ = worlds;
      ontology_ = ontology;
    }
  protected:
    void fillResponse(QueryOntology::Request& req, QueryOntology::Response& res)
    {
      if ((*worlds_).has(req.ctxt.world))
      {
        (*worlds_)[req.ctxt.world].connect(bind(&QueryOntologyService::onChanges, this, _1, _2, _3));
      }
      try
      {
        res.result = ontology_->queryOntology(req.ctxt.world, req.query);
        res.success = true;
      } catch (std::exception e)
      {
        res.success = false;
        res.error = e.what();
      }
    }

    void onChanges(string world_name, Header header, Invalidations invalidations)
    {
      for(const auto id : invalidations.node_ids_deleted)
        ontology_->removeIndividual(world_name, id);
      for(const auto id : invalidations.node_ids_updated)
        ontology_->updateIndividual(world_name, (*worlds_)[world_name].scene().nodes()[id]);

      for(const auto id : invalidations.situation_ids_deleted)
        ontology_->removeIndividual(world_name, id);
      for(const auto id : invalidations.situation_ids_updated)
        ontology_->updateSituation(world_name, (*worlds_)[world_name].timeline().situations()[id]);
    }

    OntologyPtr ontology_;
    WorldsProxyPtr worlds_;
  };

  typedef boost::shared_ptr<QueryOntologyService> QueryOntologyServicePtr;
  typedef boost::weak_ptr<QueryOntologyService> QueryOntologyServiceWeakPtr;
  typedef boost::shared_ptr<QueryOntologyService const> QueryOntologyServiceConstPtr;

  // class OntologyClient
  // {
  // public:
  //   OntologyClient(NodeHandlePtr nh)
  //   {
  //     ctx_ = boost::make_shared<UnderworldsProxy>(nh, "uwds_ontology", UNDEFINED);
  //     ontology_ = boost::make_shared<Ontology>(nh);
  //     ontology_service_ = boost::make_shared<QueryOntologyService>(nh, ctx_->client_, ctx_->worlds_proxy_, ontology_);
  //   }
  // protected:
  //   UnderworldsProxyPtr ctx_;
  //   OntologyPtr ontology_;
  //   QueryOntologyServiceWeakPtr ontology_service_;
  // };

}

#endif
