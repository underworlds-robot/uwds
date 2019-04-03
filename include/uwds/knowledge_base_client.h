// class KnowledgeBaseClient
// {
// public:
//   KnowledgeBaseClient(NodeHandlePtr nh)
//   {
//     ctx_ = boost::make_shared<UnderworldsProxy>(nh, "uwds_knowledge_base", UNDEFINED);
//     knowledge_base_ = boost::make_shared<KnowledgeBase>(nh);
//     knowledge_base_service_ = boost::make_shared<QueryKnowledgeBaseService>(nh, ctx_->client_, ctx_->worlds_proxy_, knowledge_base_);
//   }
// protected:
//   UnderworldsProxyPtr ctx_;
//   KnowledgeBasePtr knowledge_base_;
//   QueryKnowledgeBaseServiceWeakPtr knowledge_base_service_;
// };
