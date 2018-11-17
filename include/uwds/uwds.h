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

#include <uwds_msgs/GetTopology.h>
#include <uwds_msgs/GetScene.h>
#include <uwds_msgs/GetTimeline.h>
#include <uwds_msgs/GetMesh.h>

#include <uwds_msgs/ReconfigureInputs.h>
#include <uwds_msgs/List.h>
#include <uwds_msgs/ListInContext.h>
#include <uwds_msgs/Enable.h>

#include "types/worlds.h"
#include "types/topology.h"
#include "types/filter.h"
#include "types/monitor.h"

namespace uwds {

  using namespace uwds_msgs;

  /** @brief
   * The clients type enumeration
   */
  enum ClientType {
    UNDEFINED = 0,
    READER,
    MONITOR,
    PROVIDER,
    FILTER
  };
  /** @brief
   * The clients type names
   */
  static const std::array<std::string,5> ClientTypeName{"undefined",
                                                        "reader",
                                                        "monitor",
                                                        "provider",
                                                        "filter"};

  /** @brief
   * The base class containing the Underworlds data structure.
   */
  class UwdsBase
  {
   public:
    /** @brief
     * The Default constructor.
     */
    UwdsBase() {meshes_ = boost::make_shared<Meshes>();
                worlds_ = boost::make_shared<Worlds>(meshes_);
                topology_ = boost::make_shared<Topology>();}

    ~UwdsBase() {}
    /** @brief
     * The Underworlds data structure accessor.
     */
    virtual Worlds& worlds() {return * worlds_;}

    /** @brief
     * The Underworlds topology of clients accessor.
     */
    virtual Topology& topology() {return * topology_;}

    /** @brief
     * The Underworlds meshes accessor
     */
    Meshes& meshes() {return * meshes_;}

    /** @brief
     * This method reset the local data-structure.
     */
    virtual void reset() {worlds_->reset();
                          topology_->reset();
                          meshes_->reset();}

   private:

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

  };
}

#endif
