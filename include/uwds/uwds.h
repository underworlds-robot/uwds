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
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>

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
#include <uwds_msgs/QueryInContext.h>
#include <uwds_msgs/SimpleQuery.h>

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
#include "service/meshes_service.h"
#include "service/scene_service.h"
#include "service/timeline_service.h"
#include "service/knowledge_base_service.h"
#include "service/topology_service.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds {

  typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;

  /** @brief
   * The clients type enumeration.
   */
  enum ClientType {
    UNDEFINED = Client::UNDEFINED,
    READER = Client::READER,
    MONITOR = Client::MONITOR,
    PROVIDER = Client::PROVIDER,
    FILTER = Client::FILTER
  };
  /** @brief
   * The clients type names.
   */
  static const array<string,5> ClientTypeName{"undefined",
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
    UnderworldsProxy(NodeHandlePtr nh, NodeHandlePtr pnh, string client_name, ClientType client_type);

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

    //Client& client();

    string name();

  private:

    /** @brief
     * The ROS node handle shared pointer.
     */
    NodeHandlePtr nh_;

    /** @brief
     * The ROS private node handle shared pointer.
     */
    NodeHandlePtr pnh_;

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
      Underworlds(NodeHandlePtr nh, NodeHandlePtr pnh);

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

       string name();

       /** @brief
        * This method is called when changes are received.
        */
       void changesCallback(const ChangesInContextStampedPtr& msg);

       bool advertiseConnection(AdvertiseConnection::Request &req,
                                AdvertiseConnection::Response &res);

     private:

       ClientPtr client_;

       /** @brief
        * The ROS node handle shared pointer.
        */
      NodeHandlePtr nh_;

      /** @brief
       * The ROS private node handle shared pointer.
       */
     NodeHandlePtr pnh_;

      /** @brief
       * A flag to know if verbose.
       */
      bool verbose_ = false;

      int publisher_buffer_size_ = 20;

      int subscriber_buffer_size_ = 20;

      /** @brief
       * The Underworlds namespace.
       */
      string name_;

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
      GetTopologyServicePtr get_topology_service_;

      /** @brief
       * The GetScene service server
       */
      GetSceneServicePtr get_scene_service_;

      /** @brief
       * The GetTimeline service server
       */
      GetTimelineServicePtr get_timeline_service_;

      /** @brief
       * The get mesh service server
       */
      GetMeshServicePtr get_mesh_service_;

      /** @brief
       * The push mesh service server
       */
      PushMeshServicePtr push_mesh_service_;

      /** @brief
       * The push mesh service server
       */
      ros::ServiceServer advertise_service_server_;

      /** @brief
       * The changes subscribers map by world name.
       */
      map<string, boost::shared_ptr<ros::Subscriber>> changes_subscribers_map_;
  };

  typedef boost::shared_ptr<Underworlds> UnderworldsPtr;
  typedef boost::shared_ptr<Underworlds const> UnderworldsConstPtr;

}

#endif
