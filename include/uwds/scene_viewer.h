#ifndef SCENE_VIEWER_HPP
#define SCENE_VIEWER_HPP

#include "reconfigurable_client.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;
using namespace visualization_msgs;
using namespace jsk_recognition_msgs;

namespace uwds
{
  class SceneViewer : public ReconfigurableClient
  {
  public:
    /**@brief
     * The constructor.
     */
    SceneViewer(): ReconfigurableClient(READER) {}

    /**@brief
     * The default destructor
     */
    ~SceneViewer() = default;

    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit();
  protected:
    /**@brief
     * Timer to publish the tf frames.
     */
    virtual void onTimer(const ros::TimerEvent& event);

    void onReconfigure(const vector<string>& inputs);

    void onChanges(const std::string& world,
                         const std_msgs::Header& header,
                         const Invalidations& invalidations) {}

    /** @brief
     * Publish visualisation topics.
     */
    virtual void publishVisualization(const std::string world, const ros::Time stamp);

    virtual std::vector<visualization_msgs::Marker> nodeToMarkers(const std::string world, const Node node, const ros::Time stamp);

    virtual jsk_recognition_msgs::BoundingBox nodeToBoundingBox(const std::string world, const Node node, const ros::Time stamp);

    virtual sensor_msgs::CameraInfo nodeToCameraInfo(const std::string world, const Node node, const ros::Time stamp);

    /** @brief
     * The last marker ID used.
     */
    unsigned int last_marker_id_ = 0;

    /** @brief
     * The marker ID map by mesh ID.
     */
    map<string, unsigned int> marker_id_map_;

    /** @brief
     * The visualization timer.
     */
    ros::Timer publisher_timer_;

    /** @brief
     * The tf broadcaster.
     */
    tf::TransformBroadcaster tf_broadcaster_;

    /** @brief
     * The tf listener.
     */
    tf::TransformListener tf_listener_;

    map<string, boost::shared_ptr<Marker>> marker_map_;

    /** @brief
     * The marker map by node world.
     */
    map<string, boost::shared_ptr<ros::Publisher>> markers_publisher_map_;

    /** @brief
     * The bboxes map by world.
     */
    map<string, boost::shared_ptr<ros::Publisher>> bboxes_publisher_map_;

    /** @brief
     * The cameras map by world+node.id.
     */
    map<string, boost::shared_ptr<ros::Publisher>> camera_publishers_map_;
  };
}

#endif
