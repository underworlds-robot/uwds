#ifndef UWDS_SERVER_NODELET_HPP
#define UWDS_SERVER_NODELET_HPP

#include "uwds/uwds.h"
#include "uwds/dynamic_connection_based_nodelet.h"

namespace uwds
{
  #define DEFAULT_CLEAN_UP_TIMER_DURATION 5.0
  #define DEFAULT_SITUATIONS_BUFFER_SIZE 300.0

   /**@brief
    * This nodelet allow to centralize and broadcast the changes while serving
    * Underworlds data structures to the clients for initialization.
    *
    * ### Parameters
    * - '~clean_up_timer_duration' duration of the clean up timer (default : 5)
    * - '~situations_buffer_size' duration of the situations buffer (default : 300)
    * - '~use_multithread_callback' if true use multithread callback (default : true)
    * - '~publisher_buffer_size' the publishers buffer size (default : 10)
    * - '~subscriber_buffer_size' the subscribers buffer size (default : 10)
    * - '~time_synchronizer_buffer_size' the time sync buffer size (default : 10)
    *
    * ### Subscribed topics
    * - 'uwds/changes' allow to receive the changes from clients
    *
    * ### Publisher topics
    * - publish to the 'world/changes' of the worlds
    *
    * ### Advertised services
    * - 'uwds/connect_input' allow to update the clients topology
    * - 'uwds/disconnect_input' allow to update the clients topology
    * - 'uwds/connect_output' allow to update the clients topology
    * - 'uwds/disconnect_output' allow to update the clients topology
    * - 'uwds/get_topology' allow to fetch the clients topology
    * - 'uwds/get_scene' allow to fetch the scene
    * - 'uwds/get_mesh' allow to fetch the meshes
    * - '~input_worlds' list the input worlds
    * - '~output_worlds' list the output worlds
    *
    */
  class UwdsServerNodelet : public DynamicConnectionBasedNodelet,
                            public UwdsBase
  {
  public:

    /**@brief
     * The default constructor
     */
    UwdsServerNodelet() = default;

    /**@brief
     * The default destructor
     */
    ~UwdsServerNodelet() = default;

  	/** @brief
     * Initialize nodehandles nh_ and pnh_. Subclass should call
     * this method in its onInit method.
     */
    virtual void onInit();

  protected:

    //virtual bool onClientConnection();

    //virtual bool onClientDisconnection();

    /** @brief
     * This method is called by clients when request topology.
     *
     * @param req The service request
     * @param res The service response
     */
    virtual bool getTopology(uwds_msgs::GetTopology::Request& req,
                             uwds_msgs::GetTopology::Response& res);

    // Nodes related services
    /** @brief
     * This method is called when a client initialize a scene.
     *
     * @param req The service request
     * @param res The service response
     */
    virtual bool getScene(uwds_msgs::GetScene::Request& req,
                          uwds_msgs::GetScene::Response& res);

    /** @brief
     * This method is called when a client initialize a timeline.
     *
     * @param req The client request
     * @param res The server response
     */
    virtual bool getTimeline(uwds_msgs::GetTimeline::Request& req,
                             uwds_msgs::GetTimeline::Response& res);

    /** @brief
     * This method is called when a get mesh request is received.
     *
     * @param req The service request
     * @param res The service response
     */
    bool getMesh(uwds_msgs::GetMesh::Request& req,
                 uwds_msgs::GetMesh::Response& res);


   /** @brief
    * This method is called when a client connect to an input world.
    *
    * @param req The service request
    * @param res The service response
    */
    bool connectInput(uwds_msgs::Connect::Request& req,
                      uwds_msgs::Connect::Response& res);

    /** @brief
     * This method is called when a client disconnect from an input world.
     *
     * @param req The service request
     * @param res The service response
     */
    bool disconnectInput(uwds_msgs::Connect::Request& req,
                         uwds_msgs::Connect::Response& res);

   /** @brief
    * This method is called when a client connect to an output world.
    *
    * @param req The service request
    * @param res The service response
    */
   bool connectOutput(uwds_msgs::Connect::Request& req,
                      uwds_msgs::Connect::Response& res);

   /** @brief
    * This method is called when a client disconnect from an output world.
    *
    * @param req The service request
    * @param res The service response
    */
   bool disconnectOutput(uwds_msgs::Connect::Request& req,
                        uwds_msgs::Connect::Response& res);

    /** @brief
     * This method is called when a changes in a world are received.
     *
     * @param msg The message received
     */
    virtual void changesCallback(const uwds_msgs::ChangesInContextStampedPtr& msg);

    /** @brief
     * This method distribute the changes to clients.
     *
     * @param world The world to change
     * @param changes The changes to distribute
     */
    virtual void distributeChanges(const std::string& world, const Changes& changes);

    /** @brief
     * This method distribute the changes to clients.
     *
     * @param changes The changes to distribute
     */
    virtual void distributeChanges(const ChangesInContextStampedPtr& changes);

    /** @brief
     * This method is called periodically to clean up old data in the structure.
     *
     * @param event The timer event
     */
    virtual void cleanUpTimerCallback(const ros::TimerEvent& event);

    /** @brief
     * This method add a changes publisher for the given world.
     *
     * @param world The given world.
     */
    virtual void addChangesPublisher(const std::string& world);

    /** @brief
     * This method remove a changes publisher for the given world.
     *
     * @param world The given world.
     */
    virtual void removeChangesPublisher(const std::string& world);

  private:

    /** @brief
     * Timer instance to clean up old data
     */
    ros::Timer situations_clean_up_timer_;

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
     * The connect input service server,
     * allow the server to build the clients topology.
     */
    ros::ServiceServer connect_input_service_server_;

    /** @brief
     * The disconnect input service server,
     * allow the server to build the clients topology.
     */
    ros::ServiceServer disconnect_input_service_server_;

    /** @brief
     * The connect input service server,
     * allow the server to build the clients topology.
     */
    ros::ServiceServer connect_output_service_server_;

    /** @brief
     * The disconnect input service server,
     * allow the server to build the clients topology.
     */
    ros::ServiceServer disconnect_output_service_server_;
  };
}

#endif
