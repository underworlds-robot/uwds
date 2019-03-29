#ifndef WORLD_MERGER_HPP
#define WORLD_MERGER_HPP

#include "uwds/reconfigurable_client.h"
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/TransformStamped.h>
#include <pose_cov_ops/pose_cov_ops.h>

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

typedef boost::shared_ptr<tf2_ros::Buffer> BufferPtr;
typedef boost::shared_ptr<tf2_ros::TransformListener> TransformListenerPtr;

namespace uwds
{
  class WorldMerger : public uwds::ReconfigurableClient
  {
  public:
    /**@brief
     * The default constructor.
     */
    WorldMerger(): uwds::ReconfigurableClient(uwds::FILTER) {}

    /**@brief
     * The default destructor
     */
    ~WorldMerger() = default;

    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit();

  protected:

    /** @brief
     * This method is called when there is a change in a world.
     *
     * @param world The world that have been updated
     * @param header The header
     * @param invalidations The invalidations received
     */
    virtual void onChanges(const string& world,
                           const Header& header,
                           const Invalidations& invalidations);

    virtual void onReconfigure(const std::vector<std::string>& worlds) {}

    void onTimer(const ros::TimerEvent& event);

    mutex changes_mutex_;

    Changes changes_to_send_;

    TransformListenerPtr tf_listener_;

    BufferPtr tf_buffer_;
  };
}

#endif
