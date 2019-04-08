#ifndef WORLD_PROXY_HPP
#define WORLD_PROXY_HPP

#include "../tools/model_loader.h"
#include <functional>
#include "std_msgs/Header.h"
#include "uwds_msgs/Invalidations.h"
#include "uwds/types/topology.h"
#include "meshes_proxy.h"
#include "scene_proxy.h"
#include "timeline_proxy.h"
#include "knowledge_base_proxy.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds {

  using onChangesFcn = void(string, std_msgs::Header, uwds_msgs::Invalidations);

  class AdvertiseConnectionProxy : public ServiceProxy<AdvertiseConnection, ConnectionInteractionType, ConnectionActionType>
  {
  public:
    AdvertiseConnectionProxy(NodeHandlePtr nh, NodeHandlePtr pnh, ClientPtr client, string world_name):ServiceProxy<AdvertiseConnection, ConnectionInteractionType, ConnectionActionType>(nh, pnh, client, "uwds/advertise_connection")
    {
      world_name_ = world_name;
    }
  protected:
    AdvertiseConnection fillRequest(ConnectionInteractionType type, ConnectionActionType action)
    {
      AdvertiseConnection advertise_connection_srv;
      advertise_connection_srv.request.connection.ctxt.client = *client_;
      advertise_connection_srv.request.connection.ctxt.world = world_name_;
      advertise_connection_srv.request.connection.type = type;
      advertise_connection_srv.request.connection.action = action;
      return advertise_connection_srv;
    }

    string world_name_;
  };

  typedef boost::shared_ptr<AdvertiseConnectionProxy> AdvertiseConnectionProxyPtr;
  typedef boost::shared_ptr<AdvertiseConnectionProxy const> AdvertiseConnectionProxyConstPtr;

  class WorldProxy
  {
  public:
    WorldProxy(NodeHandlePtr nh, NodeHandlePtr pnh, ClientPtr client, MeshesProxyPtr meshes_proxy, string world_name)
    {
      nh_ = nh;
      pnh_ = pnh;
      client_ = client;
      world_name_ = world_name;

      meshes_proxy_ = meshes_proxy;
      scene_proxy_ = boost::make_shared<SceneProxy>(nh_, pnh_, client_, world_name_, meshes_proxy_);
      timeline_proxy_ = boost::make_shared<TimelineProxy>(nh_, pnh_, client_, world_name_);
      knowledge_base_proxy_ = boost::make_shared<KnowledgeBaseProxy>(nh_, pnh_, client_, world_name_);

      advertise_connection_proxy_ = boost::make_shared<AdvertiseConnectionProxy>(nh_, pnh_, client_, world_name_);
      scene_proxy_->getSceneFromRemote();
      timeline_proxy_->getTimelineFromRemote();

      int subscriber_buffer_size;
      pnh_->param<int>("subscriber_buffer_size", subscriber_buffer_size, 20);
      changes_subscriber_ = boost::make_shared<ros::Subscriber>(nh_->subscribe(world_name_+"/changes",
                                                                subscriber_buffer_size,
                                                                &WorldProxy::changesCallback,
                                                                this));
      int publisher_buffer_size;
      pnh_->param<int>("publisher_buffer_size", publisher_buffer_size, 20);
      changes_publisher_ = boost::make_shared<ros::Publisher>(nh_->advertise<ChangesInContextStamped>(world_name_+"/changes", publisher_buffer_size));
    }

    ~WorldProxy() {advertiseConnectionToRemote(type_, DISCONNECT);}
    /** @brief
     * The meshes accessor.
     */
    Meshes& meshes() {return meshes_proxy_->meshes();}

    /** @brief
     * The scene accessor.
     */
    Scene& scene() {return scene_proxy_->scene();}

    /** @brief
     * The timeline accessor.
     */
    Timeline& timeline() {return timeline_proxy_->timeline();}

    /** @brief
     *
     */
    vector<string> operator[](const string& query)
    {
      return knowledge_base_proxy_->queryKnowledgeBase(query);
    }

    /** @brief
     * This method is called to connect a function pointer to the world changes.
     *
     * @param filename : The filename.
     */
    bool connect(const function<onChangesFcn>& callback)
    {
      if(!ever_send_changes_)
      {
        if(!ever_connected_)
        {
          advertiseConnectionToRemote(READ, CONNECT);
          ever_connected_ = true;
        }
        ROS_INFO("[%s::connect] Connecting callback function to <%s> changes", client_->name.c_str(), world_name_.c_str());
        onChangesPtr = callback;
        return true;
      } else {
        ROS_WARN("[%s::connect] Loop detected for world <%s> ! Ignoring callback connection", client_->name.c_str(), world_name_.c_str());
        return false;
      }
  }

    /** @brief
     * This method is called when changes are received.
     *
     * @param msg : The changes message received.
     */
    void changesCallback(const ChangesInContextStampedPtr& msg)
    {
      Invalidations invalidations;
      invalidations.node_ids_deleted = scene().remove(msg->changes.nodes_to_delete);
      invalidations.node_ids_updated = scene().update(msg->changes.nodes_to_update);
      invalidations.situation_ids_deleted = timeline().remove(msg->changes.situations_to_delete);
      invalidations.situation_ids_updated = timeline().update(msg->changes.situations_to_update);
      invalidations.mesh_ids_deleted = msg->changes.meshes_to_delete;
      meshes().remove(msg->changes.meshes_to_delete);
      invalidations.mesh_ids_updated = meshes().update(msg->changes.meshes_to_update);
      if(ever_connected_)
        onChangesPtr(world_name_, msg->header, invalidations);
    }

    /** @brief
     * This method is called to send changes.
     *
     * @param changes : The changes to send.
     */
    void update(const Changes& changes)
    {
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.frame_id = global_frame_id_;
      update(header, changes);
    }

    /** @brief
     * This method is called when changes are received.
     *
     * @param msg : The changes message received.
     */
    bool update(const std_msgs::Header& header, const Changes& changes)
    {
      if (!ever_connected_)
      {
        if(!ever_send_changes_)
        {
          advertiseConnectionToRemote(WRITE, CONNECT);
          ever_send_changes_ = true;
        }
        ChangesInContextStampedPtr msg = boost::make_shared<ChangesInContextStamped>();
        msg->ctxt.client = *client_;
        msg->ctxt.world = world_name_;
        msg->header = header;
        msg->changes = changes;
        // Publish the changes to the server using zero-copy pointer passing
        //if(verbose_)ROS_INFO("[%s::update] Send changes to world <%s>", client_->name.c_str(), world_name_.c_str());
        while(changes_publisher_->isLatched() || changes_publisher_->getNumSubscribers() < 1)
        {
          ROS_WARN("[%s::update] No subscriber connected to world <%s>, must be connected at least to the Underworlds server, waiting for it...", client_->name.c_str(), world_name_.c_str());
          ros::Duration(0.15).sleep();
        }
        changes_publisher_->publish(msg);
        return true;
      } else {
         ROS_WARN("[%s::update] Loop detected for world <%s> ! Ignoring update", client_->name.c_str(), world_name_.c_str());
         return false;
      }
    }

    /** @brief
     * This method is called to push meshes to the server.
     *
     * @param filename : The filename.
     */
    bool pushMeshesFrom3DFile(const string& filename, const vector<double>& scale, vector<Mesh>& meshes_imported, vector<double>& aabb)
    {
      if (!ever_send_changes_)
      {
        advertiseConnectionToRemote(WRITE, CONNECT);
        ever_send_changes_ = true;
      }
      ModelLoader ml;
      //vector<double> aabb;
      //vector<Mesh> meshes_imported;
      if(!ml.loadMeshes(filename, scale, meshes_imported, aabb))
      {
        ROS_ERROR("[%s::pushMeshesFrom3DFile] Error occured while loading file '%s'", client_->name.c_str(), filename.c_str());
        return false;
      }
      for (const auto& mesh : meshes_imported)
      {
        meshes_proxy_->pushMeshToRemote(mesh);
      }
      return true;
    }

    bool pushMeshesFrom3DFile(const string& filename, vector<Mesh>& meshes_imported, vector<double>& aabb)
    {
      if (!ever_send_changes_)
      {
        advertiseConnectionToRemote(WRITE, CONNECT);
        ever_send_changes_ = true;
      }
      uwds::ModelLoader ml;
      vector<double> scale;
      scale.push_back(1.0);
      scale.push_back(1.0);
      scale.push_back(1.0);
      if(!ml.loadMeshes(filename, scale, meshes_imported, aabb))
      {
        ROS_ERROR("[%s::pushMeshesFrom3DFile] Error occured while loading file '%s'", client_->name.c_str(), filename.c_str());
        return false;
      }
      for (const auto& mesh : meshes_imported)
      {
        meshes_proxy_->pushMeshToRemote(mesh);
      }
      return true;
    }

    /** @brief
     * This method is called to push a scene to the server from a 3D file.
     *
     * @param filename : The filename.
     */
    bool pushSceneFrom3DFile(const string& filename)
    {
      if (!ever_send_changes_)
      {
        advertiseConnectionToRemote(WRITE, CONNECT);
        ever_send_changes_ = true;
      }
      uwds::ModelLoader ml;
      vector<Mesh> meshes_imported;
      vector<Node> nodes_imported;
      if(!ml.loadScene(filename, scene().rootID(), true, meshes_imported, nodes_imported))
      {
        ROS_ERROR("[%s::pushSceneFrom3DFile] Error occured while loading file '%s'", client_->name.c_str(), filename.c_str());
        return false;
      }
      for (const auto mesh : meshes_imported)
      {
        meshes_proxy_->pushMeshToRemote(mesh);
      }
      Changes changes;
      changes.nodes_to_update = nodes_imported;
      update(changes);
      return true;
    }

    bool pushRobotMeshesFromURDF(const string& filename, const string& primitives_folder, vector<Node>& nodes_imported, const string& root_id="")
    {
      if (!ever_send_changes_)
      {
        advertiseConnectionToRemote(WRITE, CONNECT);
        ever_send_changes_ = true;
      }
      uwds::ModelLoader ml;
      vector<Mesh> meshes_imported;
      if(!ml.loadURDF(filename, primitives_folder, root_id, meshes_imported, nodes_imported))
      {
        ROS_ERROR("[%s::pushSceneFrom3DFile] Error occured while loading URDF file '%s'", client_->name.c_str(), filename.c_str());
        return true;
      }
      for (const auto mesh : meshes_imported)
      {
        meshes_proxy_->pushMeshToRemote(mesh);
      }
      return true;
    }

    bool advertiseConnectionToRemote(const ConnectionInteractionType& type, const ConnectionActionType& action)
    {
      AdvertiseConnection advertise_connection_srv = advertise_connection_proxy_->call(type, action);
      if (!advertise_connection_srv.response.success)
        ROS_ERROR("[%s::advertiseConnectionToRemote] Error occured when processing '%s' : %s",
            advertise_connection_proxy_->client().name.c_str(),
            advertise_connection_proxy_->serviceName().c_str(),
            advertise_connection_srv.response.error.c_str());
      return advertise_connection_srv.response.success;
    }


  private:

    bool verbose_;

    string world_name_;

    NodeHandlePtr nh_;

    NodeHandlePtr pnh_;

    ClientPtr client_;

    /* @brief
     *  The interaction type.
     */
    ConnectionInteractionType type_ = READ;

    /** @brief
     * A flag to if ever send changes.
     */
    bool ever_connected_ = false;

    /** @brief
     * A flag to if ever send changes.
     */
    bool ever_send_changes_ = false;

    /** @brief
     * The global frame id.
     */
    string global_frame_id_;

    MeshesProxyPtr meshes_proxy_;

    SceneProxyPtr scene_proxy_;

    TimelineProxyPtr timeline_proxy_;

    KnowledgeBaseProxyPtr knowledge_base_proxy_;

    AdvertiseConnectionProxyPtr advertise_connection_proxy_;

    /** @brief
     * The GetTimeline service client.
     */
    function<onChangesFcn> onChangesPtr = 0;

    /** @brief
     * The changes subscriber shared pointer.
     */
    boost::shared_ptr<ros::Subscriber> changes_subscriber_;

    /** @brief
     * The changes publisher shared pointer.
     */
    boost::shared_ptr<ros::Publisher> changes_publisher_;

  };

  typedef boost::shared_ptr<WorldProxy> WorldProxyPtr;
  typedef boost::shared_ptr<WorldProxy const> WorldProxyConstPtr;

}

#endif
