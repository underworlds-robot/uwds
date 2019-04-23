#include "uwds/uwds.h"
using namespace std;
using namespace uwds_msgs;

namespace uwds
{

///////////////////////////////////////////////////////////////////////////////
//////////// Underworlds Proxy
///////////////////////////////////////////////////////////////////////////////

  UnderworldsProxy::UnderworldsProxy(NodeHandlePtr nh, NodeHandlePtr pnh, std::string client_name, ClientType client_type)
  {
    nh_ = nh;
    pnh_ = pnh;
    client_ = boost::make_shared<Client>();
    client_->name = client_name;
    client_->id = NEW_UUID;
    client_->type = client_type;

    meshes_proxy_ = boost::make_shared<MeshesProxy>(nh_, pnh_, client_);
    worlds_proxy_ = boost::make_shared<WorldsProxy>(nh_, pnh_, client_, meshes_proxy_);
    topology_proxy_ = boost::make_shared<TopologyProxy>(nh_, pnh_, client_);

    ROS_INFO("[%s::init] Underworlds client ready !", client_->name.c_str());
  }

  WorldsProxy& UnderworldsProxy::worlds() {return * worlds_proxy_;}

  Topology& UnderworldsProxy::topology()
  {
    topology_proxy_->getTopologyFromRemote();
    return topology_proxy_->topology();
  }

  Meshes& UnderworldsProxy::meshes() {return meshes_proxy_->meshes();}

  string UnderworldsProxy::name() {return client_->name;}

  ///////////////////////////////////////////////////////////////////////////////
  //////////// Underworlds Server
  ///////////////////////////////////////////////////////////////////////////////

  Underworlds::Underworlds(NodeHandlePtr nh, NodeHandlePtr pnh)
  {
    nh_ = nh;
    pnh_ = pnh;
    client_ = boost::make_shared<Client>();
    client_->name = "uwds";
    client_->id = NEW_UUID;
    client_->type = UNDEFINED;
    name_ = "uwds";
    if(verbose_)ROS_INFO("[%s::init] Create Underworlds data-structures...", client_->name.c_str());
    meshes_ = boost::make_shared<Meshes>();
    worlds_ = boost::make_shared<Worlds>(meshes_);
    topology_ = boost::make_shared<Topology>();
    if(verbose_)ROS_INFO("[%s::init] Advertise services...", client_->name.c_str());

    get_topology_service_ = boost::make_shared<GetTopologyService>(nh_, client_, topology_);

    push_mesh_service_ = boost::make_shared<PushMeshService>(nh_, client_, meshes_);
    get_mesh_service_ = boost::make_shared<GetMeshService>(nh_, client_, meshes_);

    get_timeline_service_ = boost::make_shared<GetTimelineService>(nh_, client_, worlds_);

    get_scene_service_ = boost::make_shared<GetSceneService>(nh_, client_, worlds_);



    advertise_service_server_ = nh_->advertiseService("uwds/advertise_connection",
                                      &Underworlds::advertiseConnection,
                                      this);

    pnh_->param<int>("subscriber_buffer_size", subscriber_buffer_size_, 30);
    pnh_->param<int>("publisher_buffer_size", publisher_buffer_size_, 20);
    pnh_->param<bool>("verbose", verbose_, true);

    ROS_INFO("[%s::init] Underworlds server ready !", client_->name.c_str());
  }

  string Underworlds::name() {return client_->name;}

  void Underworlds::changesCallback(const ChangesInContextStampedPtr& msg)
  {
     float delay = (ros::Time::now() - msg->header.stamp).toSec();

     if(msg->ctxt.world == "uwds")
     {
       throw std::runtime_error("World namespace reserved.");
     }
     if(msg->ctxt.world == "")
     {
       throw std::runtime_error("Empty world namespace.");
     }
     try
     {
       auto& scene = worlds()[msg->ctxt.world].scene();
       auto& timeline = worlds()[msg->ctxt.world].timeline();

       scene.nodes().remove(msg->changes.nodes_to_delete);
       scene.nodes().update(msg->changes.nodes_to_update);

       timeline.situations().remove(msg->changes.situations_to_delete);
       timeline.situations().update(msg->changes.situations_to_update);

       meshes().remove(msg->changes.meshes_to_delete);
       meshes().update(msg->changes.meshes_to_update);

     } catch(const std::exception& e) {
       ROS_ERROR("[%s::changesCallback] Exception occured when receiving changes from <%s> : %s",
                     name_.c_str(),
                     msg->ctxt.world.c_str(),
                     e.what());
     }
  }

  bool Underworlds::advertiseConnection(AdvertiseConnection::Request &req,
                           AdvertiseConnection::Response &res)
  {
    if(verbose_)ROS_INFO("[%s::connectInput] Client <%s> requested 'uwds/advertise_connection' in <%s> world",
                                 name_.c_str(),
                                 req.connection.ctxt.client.name.c_str(),
                                 req.connection.ctxt.world.c_str());
    try
    {
      if(req.connection.ctxt.world == "uwds")
      {
        throw std::runtime_error("World namespace <uwds> reserved.");
      }
      if(req.connection.ctxt.world == "")
      {
        throw std::runtime_error("Empty world namespace.");
      }

      if(changes_subscribers_map_.count(req.connection.ctxt.world) == 0)
      {
        changes_subscribers_map_.emplace(req.connection.ctxt.world, boost::make_shared<ros::Subscriber>(nh_->subscribe(req.connection.ctxt.world+"/changes", subscriber_buffer_size_, &Underworlds::changesCallback, this)));
        if(verbose_)ROS_INFO("[%s::advertiseConnection] Changes subscriber for world <%s> created", name_.c_str(), req.connection.ctxt.world.c_str());
      }

      topology().update(req.connection.ctxt, (ConnectionInteractionType) req.connection.type, (ConnectionActionType) req.connection.action);

      res.success = true;
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("[%s::advertiseConnection] Exception occured while registering the <%s> client connection to world <%s> : %s",
                         req.connection.ctxt.client.name.c_str(),
                         name_.c_str(),
                         req.connection.ctxt.world.c_str(),
                         e.what());
      res.success = false;
      res.error = e.what();
    }
    return true;
  }



}
