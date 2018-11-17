#ifndef DYNAMIC_CONNECTION_BASED_NODELET_HPP
#define DYNAMIC_CONNECTION_BASED_NODELET_HPP

#include "uwds/uwds.h"
#include <nodelet/nodelet.h>

namespace uwds
{
  #define DEFAULT_SUBSCRIBER_BUFFER_SIZE 10
  #define DEFAULT_PUBLISHER_BUFFER_SIZE 10
  #define DEFAULT_TIME_SYNCHRONIZER_BUFFER_SIZE 10

   /**@brief
    * This base nodelet allow to handle the dynamic connections.
    *
    * ### Parameters
    * - '~use_multithread_callback' if true use multithread callback (default : true)
    * - '~publisher_buffer_size' the publishers buffer size (default : 10)
    * - '~subscriber_buffer_size' the subscribers buffer size (default : 10)
    * - '~time_synchronizer_buffer_size' the time sync buffer size (default : 10)
    *
    * ### Subscribed topics
    * - none
    *
    * ### Publisher topics
    * - none
    *
    * ### Advertised services
    * - '~input_worlds' list the input worlds
    * - '~output_worlds' list the output worlds
    *
    */
  class DynamicConnectionBasedNodelet : public nodelet::Nodelet
  {
  public:
    /** @brief
     * Enum to represent configuration status.
     */
    enum ConnectionStatus
    {
      NOT_INITIALIZED,
      NOT_CONNECTED,
      CONNECTING,
      CONNECTED
    };

    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit();

  protected:
    /** @brief
     * This method reset the list that track the active input connections.
     */
    virtual void resetInputWorlds();

    /** @brief
     * This method reset the list that track the active output connections.
     */
    virtual void resetOutputWorlds();

    /** @brief
     * This method add an entry to the input connection track list.
     */
    virtual void addInputWorld(const std::string& world);

    /** @brief
     * This method add an entry to the output connection track list.
     */
    virtual void addOutputWorld(const std::string& world);

    /** @brief
     * This method is called when the service ~list_input_connections is requested.
     */
    virtual bool listInputWorlds(uwds_msgs::List::Request &req,
                                 uwds_msgs::List::Response &res);

    /** @brief
     * This method is called when the service ~list_output_connections is requested.
     */
    virtual bool listOutputWorlds(uwds_msgs::List::Request &req,
                                  uwds_msgs::List::Response &res);


    /** @brief
     * A flag to know if verbose,
     * or not.
     */
    bool verbose_;

   /** @brief
    * The nodelet name.
    */
    std::string nodelet_name_;
    /** @brief
     * The publishers buffer size.
     */
    int publisher_buffer_size_;

    /** @brief
     * The subscribers buffer size.
     */
    int subscriber_buffer_size_;

    /** @brief
     * The time synchronizer buffer size.
     */
    int time_synchronizer_buffer_size_;

    /** @brief
     * The connection mutex.
     */
    std::mutex connection_mutex_;

    /** @brief
     * The list that track the active input worlds.
     */
    std::vector<std::string> input_worlds_;

    /** @brief
     * The list that track the active output worlds.
     */
    std::vector<std::string> output_worlds_;

    /** @brief
     * The changes subscriber.
     */
    boost::shared_ptr<ros::Subscriber> changes_subscriber_;

    /** @brief
     * The changes publisher.
     */
    boost::shared_ptr<ros::Publisher> changes_publisher_;

    /** @brief
     * The changes subscribers map by world name.
     */
    std::map<std::string, boost::shared_ptr<ros::Subscriber>> changes_subscribers_map_;

    /** @brief
     * The synchronized changes subscribers map by world name.
     */
    std::map<std::string, boost::shared_ptr<message_filters::Subscriber<uwds_msgs::ChangesInContextStamped>>> sync_changes_subscribers_map_;

    /** @brief
     * The changes publishers map by world name.
     */
    std::map<std::string, boost::shared_ptr<ros::Publisher>> changes_publishers_map_;

    /** @brief
     * A flag to check if ever connected
     * or not.
     */
    bool ever_connected_;

    /** @brief
     * Status of the input configuration.
     */
    ConnectionStatus connection_status_;

    /** @brief
     * The service server to list the input connections.
     */
    ros::ServiceServer list_input_worlds_server_;

    /** @brief
     * The service server to list the output connections.
     */
    ros::ServiceServer list_output_worlds_server_;

    /** @brief
     * Shared pointer to nodehandle.
     */
    boost::shared_ptr<ros::NodeHandle> nh_;

    /** @brief
     * Shared pointer to private nodehandle.
     */
    boost::shared_ptr<ros::NodeHandle> pnh_;

  };
}

#endif
