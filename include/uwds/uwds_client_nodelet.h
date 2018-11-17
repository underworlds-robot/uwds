#ifndef UWDS_CLIENT_NODELET_HPP
#define UWDS_CLIENT_NODELET_HPP

#include "uwds/dynamic_connection_based_nodelet.h"

namespace uwds
{
  /**@brief
   * This nodelet allow to request data and send changes to the server.
   * Underworlds clients must inherit from this class.
   *
   * ### Parameters
   * - '~use_scene' if true use the scene (default : true)
   * - '~use_timeline' if true use the timeline (default : true)
   * - '~use_meshes' if true use the meshes (default : true)
   * - '~use_multithread_callback' if true use multithread callback (default : true)
   * - '~publisher_buffer_size' the publishers buffer size (default : 10)
   * - '~subscriber_buffer_size' the subscribers buffer size (default : 10)
   * - '~time_synchronizer_buffer_size' the time sync buffer size (default : 10)
   *
   * ### Subscribed topics
   * - none
   *
   * ### Publisher topics
   * - 'uwds/changes' allow to send updates to the server
   *
   * ### Advertised services
   * - '~input_worlds' list the input worlds
   * - '~output_worlds' list the output worlds
   *
   */
  class UwdsClientNodelet : public DynamicConnectionBasedNodelet,
                            public UwdsBase
  {
  public:

    /**@brief
     * The default constructor
     */
    UwdsClientNodelet(): client_type_(UNDEFINED) {}

    /**@brief
     * The constructor with the client type
     *
     * @param client_type The client type
     */
    UwdsClientNodelet(uwds::ClientType client_type): client_type_(client_type) {}

    /**@brief
     * The default destructor
     */
    ~UwdsClientNodelet() = default;

    /** @brief
     * Initialize nodehandles nh_ and pnh_. Subclass should call
     * this method in its onInit method.
     */
    virtual void onInit();

  protected:

    /** @brief
     * This method send the changes to the server.
     *
     * @param world The world to update
     * @param header The header
     * @param changes The changes to send
     */
    void sendWorldChanges(const std::string world,
                          const std_msgs::Header header,
          								const uwds_msgs::Changes changes);

    /** @brief
     * This method is called to request the Underworlds clients topology.
     */
    void getTopologyFromRemote();

    /** @brief
     * This method is called to request the Underworlds scene.
     *
     * @param world The world to connect
     */
    void getSceneFromRemote(const std::string& world);

    /** @brief
     * This method is called to request the Underworlds timeline.
     *
     * @param world The world to connect
     */
    void getTimelineFromRemote(const std::string& world);

    /** @brief
     * This method is called when a world is initialized.
     *
     * @param world The world to initialize
     */
    void initializeWorld(const std::string& world);

    /** @brief
     * This method is called to request a mesh from the server.
     *
     * @param mesh_id The mesh ID to get
     */
    bool getMeshFromRemote(const std::string& mesh_id);

    /** @brief
     * This method is called to request the meshes of a given node.
     *
     * @param node The node to process
     * @return The list of mesh IDs
     */
    std::vector<std::string> getNodeMeshes(const uwds_msgs::Node& node);


  protected:
    /** @brief
     * A flag to know if the scene is used,
     * or not.
     */
    bool use_scene_;
    /** @brief
     * A flag to know if the timeline is used,
     * or not.
     */
    bool use_timeline_;

    /** @brief
     *A flag to know if the meshes are used,
     * or not.
     */
    bool use_meshes_;

  private:
    /** @brief
     * The client unique id.
     */
    std::string client_id_;

    /** @brief
     * The client type.
     */
    uwds::ClientType client_type_;

    /** @brief
     * A flag to check if ever send changes
     * or not.
     */
    bool ever_send_changes_;

    /** @brief
     * The GetMesh service client.
     */
    ros::ServiceClient get_mesh_service_client_;

    /** @brief
     * The GetTimeline service client.
     */
    ros::ServiceClient get_timeline_service_client_;

    /** @brief
     * The GetScene service client.
     */
    ros::ServiceClient get_scene_service_client_;

    /** @brief
     * The GetTopology service client.
     */
    ros::ServiceClient get_topology_service_client_;

  };
}

#endif
