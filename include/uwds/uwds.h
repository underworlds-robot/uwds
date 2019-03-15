#ifndef UWDS_HPP
#define UWDS_HPP

#include<string>
#include<vector>
#include<map>
#include<mutex>
#include<queue>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <tf/tf.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <uwds_msgs/Client.h>
#include <uwds_msgs/ClientInteraction.h>

#include <uwds_msgs/Context.h>
#include <uwds_msgs/Property.h>
#include <uwds_msgs/Node.h>
#include <uwds_msgs/Situation.h>
#include <uwds_msgs/Invalidations.h>
#include <uwds_msgs/Changes.h>
#include <uwds_msgs/ChangesInContextStamped.h>
#include <uwds_msgs/MeshTriangle.h>
#include <uwds_msgs/Mesh.h>
#include <std_msgs/ColorRGBA.h>

#include <uwds_msgs/AdvertiseConnection.h>
#include <uwds_msgs/GetTopology.h>
#include <uwds_msgs/GetScene.h>
#include <uwds_msgs/GetTimeline.h>
#include <uwds_msgs/GetMesh.h>
#include <uwds_msgs/PushMesh.h>
#include <uwds_msgs/QueryOntology.h>

#include <uwds_msgs/ReconfigureInputs.h>
#include <uwds_msgs/List.h>
#include <uwds_msgs/ListInContext.h>
#include <uwds_msgs/Enable.h>

#include "types/worlds.h"
#include "types/topology.h"
#include "proxy/worlds_proxy.h"
#include "proxy/topology_proxy.h"
#include "proxy/meshes_proxy.h"
#include "service/service.h"
#include "service/scene_service.h"
#include "service/timeline_service.h"
#include "service/ontology_service.h"
#include "service/topology_service.h"

namespace uwds {

  using namespace std;
  using namespace uwds_msgs;

  typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;

  /** @brief
   * The clients type enumeration.
   */
  enum ClientType {
    UNDEFINED = uwds_msgs::Client::UNDEFINED,
    READER = uwds_msgs::Client::READER,
    MONITOR = uwds_msgs::Client::MONITOR,
    PROVIDER = uwds_msgs::Client::PROVIDER,
    FILTER = uwds_msgs::Client::FILTER
  };
  /** @brief
   * The clients type names.
   */
  static const std::array<std::string,5> ClientTypeName{"undefined",
                                                        "reader",
                                                        "monitor",
                                                        "provider",
                                                        "filter"};

  class UnderworldsProxy
  {
  public:
    /** @brief
     * The Underworlds proxy class (to instantiate into clients).
     */
    UnderworldsProxy(NodeHandlePtr nh, std::string client_name, ClientType client_type);

    /** @brief
     * The Underworlds data structure accessor.
     */
    WorldsProxy& worlds();

    /** @brief
     * The Underworlds topology of clients accessor.
     */
    Topology& topology();

    /** @brief
     * The Underworlds meshes accessor.
     */
    Meshes& meshes();

  private:

    /** @brief
     * The ROS node handle shared pointer.
     */
    NodeHandlePtr nh_;

    /** @brief
     * The Underworlds client description shared ptr.
     */
    ClientPtr client_;

    /** @brief
     * The Underworlds meshes proxy.
     */
    MeshesProxyPtr meshes_proxy_;

     /** @brief
      * The Underworlds main data structure proxy shared pointer.
      */
     WorldsProxyPtr worlds_proxy_;

     /** @brief
      * The Underworlds topology of clients shared pointer.
      */
     TopologyProxyPtr topology_proxy_;
  };

  typedef boost::shared_ptr<UnderworldsProxy> UnderworldsProxyPtr;
  typedef boost::shared_ptr<UnderworldsProxy const> UnderworldsProxyConstPtr;

  class Underworlds
  {
    public:
      /** @brief
       * The class containing the Underworlds server
       */
      Underworlds(NodeHandlePtr nh);

       ~Underworlds() {}
       /** @brief
        * The Underworlds data structure accessor.
        */
       Worlds& worlds() {return * worlds_;}

       /** @brief
        * The Underworlds topology of clients accessor.
        */
       Topology& topology() {return * topology_;}

       /** @brief
        * The Underworlds meshes accessor.
        */
       Meshes& meshes() {return * meshes_;}

       /** @brief
        * This method is called when changes are received.
        */
       void changesCallback(const ChangesInContextStampedPtr& msg)
       {
          float delay = (ros::Time::now() - msg->header.stamp).toSec();
          if(verbose_)ROS_INFO("[%s::changesCallback] Received changes from client <%s> in <%s> world %f in the past",
                                        name_.c_str(),
                                        msg->ctxt.client.name.c_str(),
                                        msg->ctxt.world.c_str(),
                                        delay);
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
            timeline.situations().remove(msg->changes.situations_to_delete);
            meshes().remove(msg->changes.meshes_to_delete);
            meshes().update(msg->changes.meshes_to_update);
            scene.nodes().update(msg->changes.nodes_to_update);
          } catch(const std::exception& e) {
            ROS_ERROR("[%s::changesCallback] Exception occured when receiving changes from <%s> : %s",
                          name_.c_str(),
                          msg->ctxt.world.c_str(),
                          e.what());
          }
       }

       bool getTopology(GetTopology::Request &req,
                        GetTopology::Response &res)
       {
       	if(verbose_)ROS_INFO("[%s::getTopology] Request 'uwds/get_topology'", name_.c_str());
       	try {
           topology().lock();
           for (auto client : topology().clients())
           {
             res.clients.push_back(*client);
           }
           for (auto client_interactions : topology().clientsInteractions())
           {
             for(const auto client_interaction : *client_interactions)
             {
               if (std::find(res.worlds.begin(), res.worlds.end(), client_interaction->ctxt.world) == res.worlds.end())
                  res.worlds.push_back(client_interaction->ctxt.world);
               res.client_interactions.push_back(*client_interaction);
             }
           }
           topology().unlock();
       		 res.success = true;
       	} catch(const std::exception& e) {
           	ROS_ERROR("[%s::getTopology] Exception occured : %s", name_.c_str(), e.what());
           	res.success = false;
           	res.error = e.what();
            topology().unlock();
         }
         return true;
       }

       bool getScene(GetScene::Request &req,
                     GetScene::Response &res)
       {
       	 if(verbose_)ROS_INFO("[%s::getScene] Client <%s> request uwds/get_scene in <%s> world",
                                  name_.c_str(),
                                  req.ctxt.client.name.c_str(),
                                  req.ctxt.world.c_str());
         try
         {
           if(req.ctxt.world == "uwds")
           {
             throw std::runtime_error("World namespace <uwds> reserved.");
           }
           if(req.ctxt.world == "")
           {
             throw std::runtime_error("Empty world namespace.");
           }
         	 auto& scene = worlds()[req.ctxt.world].scene();
           scene.lock();
           for (auto node : scene.nodes())
         	 {
         	   res.nodes.push_back(*node);
         	 }
           scene.unlock();
         	 res.root_id = scene.rootID();
         	 res.success = true;
         	 res.error = "";
         }
         catch(const std::exception& e)
         {
         	 ROS_ERROR("[%s::getScene] Exception occured : %s", name_.c_str(), e.what());
         	 res.success = false;
         	 res.error = e.what();
         }
         return true;
       }

       bool getTimeline(GetTimeline::Request &req,
                        GetTimeline::Response &res)
       {
       	 if(verbose_)ROS_INFO("[%s::getTimeline] Client <%s> request 'uwds/get_timeline' in <%s> world", name_.c_str(), req.ctxt.client.name.c_str(), req.ctxt.world.c_str());
         try
         {
            if(req.ctxt.world == "uwds")
            {
              throw std::runtime_error("World namespace <uwds> reserved.");
            }
            if(req.ctxt.world == "")
            {
              throw std::runtime_error("Empty world namespace.");
            }
         	  auto& timeline = worlds()[req.ctxt.world].timeline();
         	  std::vector<uwds_msgs::Situation> situations;
            if(timeline.size() > 0)
            {
               timeline.lock();
               for (const auto& situation : timeline.situations())
             	 {
             	   situations.push_back(*situation);
             	 }
               timeline.unlock();
             }
         	res.situations = situations;
         	res.origin = timeline.origin();
         	res.success = true;
         	res.error = "";
         }
         catch(const std::exception& e)
         {
         	ROS_ERROR("[%s::getTimeline] Exception occured while sending timeline for world <%s> to client <%s> : %s",
                            name_.c_str(), req.ctxt.world.c_str(),
                            req.ctxt.client.name.c_str(),
                            e.what());
         	res.success = false;
         	res.error = e.what();
         }
         return true;
       }

       bool getMesh(GetMesh::Request &req,
                    GetMesh::Response &res)
       {
         if(verbose_)ROS_INFO("[%s::getMesh] Client request 'uwds/get_mesh' <%s>",
                                    name_.c_str(),
                                    req.mesh_id.c_str());
         try
         {
           if (meshes().has(req.mesh_id))
           {
             res.mesh = meshes()[req.mesh_id];
             res.success = true;
           } else {
             ROS_WARN("Mesh %s not existing",req.mesh_id.c_str());
             res.success = false;
             res.error = "Requested mesh <"+req.mesh_id+"> not existing";
           }
         }
         catch(const std::exception& e)
         {
           ROS_ERROR("[%s::getMesh] Exception occured while sending mesh <%s> : %s",
                        name_.c_str(),
                        req.mesh_id.c_str(), e.what());
           res.success = false;
           res.error = e.what();
         }
         return true;
       }

       bool pushMesh(PushMesh::Request &req,
                     PushMesh::Response &res)
       {
         try
         {
           if(req.mesh.id == "")
           {
             throw std::runtime_error("Invalid mesh id.");
           }
           if (!meshes().has(req.mesh.id))
           {
             meshes().update(req.mesh);
             res.success = true;
           } else {
             ROS_WARN("[%s::pushMesh] Mesh <%s> already existing",req.mesh.id.c_str(), req.mesh.id.c_str());
             res.success = false;
             res.error = "Pushed mesh <"+req.mesh.id+"> already existing";
           }
         }
         catch(const std::exception& e)
         {
           ROS_ERROR("[%s::pushMesh] Exception occured while sending mesh <%s> : %s",
                        name_.c_str(),
                        req.mesh.id.c_str(), e.what());
           res.success = false;
           res.error = e.what();
         }
         return true;
       }

       bool advertiseConnection(AdvertiseConnection::Request &req,
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

     private:

       ClientPtr client_;

       /** @brief
        * The ROS node handle shared pointer.
        */
      NodeHandlePtr nh_;

      /** @brief
       * A flag to know if verbose.
       */
      bool verbose_ = false;

      int publisher_buffer_size_ = 20;

      int subscriber_buffer_size_ = 20;

      /** @brief
       * The Underworlds namespace.
       */
      std::string name_;

      /** @brief
       * The Underworlds meshes.
       */
      MeshesPtr meshes_;

      /** @brief
       * The Underworlds main data structure.
       */
      WorldsPtr worlds_;

      /** @brief
       * The Underworlds topology of clients.
       */
      TopologyPtr topology_;

      /** @brief
       * The GetTopology service server
       */
      ros::ServiceServer get_topology_service_server_;

      /** @brief
       * The GetScene service server
       */
      ros::ServiceServer get_scene_service_server_;

      /** @brief
       * The GetTimeline service server
       */
      ros::ServiceServer get_timeline_service_server_;

      /** @brief
       * The get mesh service server
       */
      ros::ServiceServer get_mesh_service_server_;

      /** @brief
       * The push mesh service server
       */
      ros::ServiceServer push_mesh_service_server_;

      /** @brief
       * The push mesh service server
       */
      ros::ServiceServer advertise_service_server_;

      /** @brief
       * The changes subscribers map by world name.
       */
      std::map<std::string, boost::shared_ptr<ros::Subscriber>> changes_subscribers_map_;
  };

  typedef boost::shared_ptr<Underworlds> UnderworldsPtr;
  typedef boost::shared_ptr<Underworlds const> UnderworldsConstPtr;

}

#endif
