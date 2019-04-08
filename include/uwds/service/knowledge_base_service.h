#ifndef ONTOLOGY_SERVICE_HPP
#define ONTOLOGY_SERVICE_HPP

#include "service.h"
#include "uwds/proxy/worlds_proxy.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds {

  class KnowledgeBase : public enable_shared_from_this<KnowledgeBase>
  {
  public:
    KnowledgeBase(NodeHandlePtr nh)
    {
      nh_ = nh;
    }
    virtual bool updateNode(string world_name, Node node) = 0;
    virtual bool removeNode(string world_name, string node_id) = 0;
    virtual bool updateSituation(string world_name, Situation situation) = 0;
    virtual bool removeSituation(string world_name, string situation_id) = 0;
    virtual vector<string> queryKnowledgeBase(string world_name, string query) = 0;
  protected:
    NodeHandlePtr nh_;
  };

  typedef boost::shared_ptr<KnowledgeBase> KnowledgeBasePtr;
  typedef boost::weak_ptr<KnowledgeBase> KnowledgeBaseWeakPtr;
  typedef boost::shared_ptr<KnowledgeBase const> KnowledgeBaseConstPtr;

  class QueryKnowledgeBaseService : public Service<QueryInContext::Request, QueryInContext::Response>
  {
  public:
    QueryKnowledgeBaseService(NodeHandlePtr nh, ClientPtr client, WorldsProxyPtr worlds, KnowledgeBasePtr knowledge_base):Service<QueryInContext::Request, QueryInContext::Response>(nh, client, "uwds/query_knowledge_base")
    {
      worlds_ = worlds;
      knowledge_base_ = knowledge_base;
    }
  protected:
    void fillResponse(QueryInContext::Request& req, QueryInContext::Response& res)
    {
      if ((*worlds_).has(req.ctxt.world))
      {
        auto& scene = (*worlds_)[req.ctxt.world].scene();
        auto& timeline = (*worlds_)[req.ctxt.world].timeline();
        auto& meshes = (*worlds_)[req.ctxt.world].meshes();
        for(const auto& node : scene.nodes())
        {
          knowledge_base_->updateNode(req.ctxt.world, *node);
        }
        for(const auto& situation : timeline.situations())
        {
          knowledge_base_->updateSituation(req.ctxt.world, *situation);
        }
        (*worlds_)[req.ctxt.world].connect(bind(&QueryKnowledgeBaseService::onChanges, this, _1, _2, _3));
      }
      try
      {
        res.result = knowledge_base_->queryKnowledgeBase(req.ctxt.world, req.query);
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
        knowledge_base_->removeNode(world_name, id);
      for(const auto id : invalidations.node_ids_updated)
        knowledge_base_->updateNode(world_name, (*worlds_)[world_name].scene().nodes()[id]);
      for(const auto id : invalidations.situation_ids_deleted)
        knowledge_base_->removeSituation(world_name, id);
      for(const auto id : invalidations.situation_ids_updated)
        knowledge_base_->updateSituation(world_name, (*worlds_)[world_name].timeline().situations()[id]);
    }

    KnowledgeBasePtr knowledge_base_;
    WorldsProxyPtr worlds_;
  };

  typedef boost::shared_ptr<QueryKnowledgeBaseService> QueryKnowledgeBaseServicePtr;
  typedef boost::weak_ptr<QueryKnowledgeBaseService> QueryKnowledgeBaseServiceWeakPtr;
  typedef boost::shared_ptr<QueryKnowledgeBaseService const> QueryKnowledgeBaseServiceConstPtr;

}

#endif
