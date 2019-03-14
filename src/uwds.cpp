#include "uwds/uwds.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds
{

///////////////////////////////////////////////////////////////////////////////
//////////// Underworlds Proxy
///////////////////////////////////////////////////////////////////////////////

  UnderworldsProxy::UnderworldsProxy(NodeHandlePtr nh, std::string client_name, ClientType client_type)
  {
    nh_ = nh;
    client_ = boost::make_shared<Client>();
    client_->name = client_name;
    client_->id = NEW_UUID;
    client_->type = client_type;

    meshes_proxy_ = boost::make_shared<MeshesProxy>(nh_, client_);
    worlds_proxy_ = boost::make_shared<WorldsProxy>(nh_, client_, meshes_proxy_);
    topology_proxy_ = boost::make_shared<TopologyProxy>(nh_, client_);

    ROS_INFO("[%s::init] Underworlds client ready !", client_->name.c_str());
  }

  WorldsProxy& UnderworldsProxy::worlds() {return * worlds_proxy_;}

  Topology& UnderworldsProxy::topology()
  {
    topology_proxy_->getTopologyFromRemote();
    return topology_proxy_->topology();
  }

  Meshes& UnderworldsProxy::meshes() {return meshes_proxy_->meshes();}

  ///////////////////////////////////////////////////////////////////////////////
  //////////// Underworlds Server
  ///////////////////////////////////////////////////////////////////////////////

  Underworlds::Underworlds(NodeHandlePtr nh)
  {
    nh_ = nh;
    name_ = "uwds";
    meshes_ = boost::make_shared<Meshes>();
    worlds_ = boost::make_shared<Worlds>(meshes_);
    topology_ = boost::make_shared<Topology>();

    get_topology_service_server_ = nh_->advertiseService("uwds/get_topology",
                                            &Underworlds::getTopology,
                                            this);

    get_scene_service_server_ = nh_->advertiseService("uwds/get_scene",
                                        &Underworlds::getScene,
                                        this);

    get_timeline_service_server_ = nh_->advertiseService("uwds/get_timeline",
                                          &Underworlds::getTimeline,
                                          this);

    get_mesh_service_server_ = nh_->advertiseService("uwds/get_mesh",
                                      &Underworlds::getMesh,
                                      this);

    push_mesh_service_server_ = nh_->advertiseService("uwds/push_mesh",
                                      &Underworlds::pushMesh,
                                      this);

    advertise_service_server_ = nh_->advertiseService("uwds/advertise_connection",
                                      &Underworlds::advertiseConnection,
                                      this);

    nh_->param<int>("uwds/subscriber_buffer_size", subscriber_buffer_size_, 20);
    nh_->param<int>("uwds/publisher_buffer_size", publisher_buffer_size_, 20);
    nh_->param<bool>("verbose", verbose_, true);
    ROS_INFO("[%s::init] Underworlds server ready !", name_.c_str());
  }



}
