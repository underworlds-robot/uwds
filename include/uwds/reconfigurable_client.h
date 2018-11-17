#ifndef RECONFIGURABLE_CLIENT_HPP
#define RECONFIGURABLE_CLIENT_HPP

#include "uwds/uwds.h"
#include "uwds/uwds_client_nodelet.h"

using namespace uwds_msgs;

namespace uwds
{
   /**@brief
    * This Nodelet allow to dynamically reconfigure input worlds.
    * Underworlds clients that need to subscribe to worlds must inherit
    * from this class.
    *
    * You will need to implement the abstract methods in order to use this class.
    *
    * ### Parameters
    * - '~synchronized' if true, synchronize the input worlds (default : false)
    * - '~default_inputs' the default inputs of the client (default : none)
    * - '~use_scene' if true use the scene (default : true)
    * - '~use_timeline' if true use the timeline (default : true)
    * - '~use_meshes' if true use the meshes (default : true)
    * - '~use_multithread_callback' if true use multithread callback (default : true)
    * - '~publisher_buffer_size' the publishers buffer size (default : 10)
    * - '~subscriber_buffer_size' the subscribers buffer size (default : 10)
    * - '~time_synchronizer_buffer_size' the time sync buffer size (default : 10)
    *
    * ### Subscribed topics
    * - subscribe to the 'world/changes' of the connected worlds
    *
    * ### Publisher topics
    * - 'uwds/changes' allow to send updates to the server
    *
    * ### Advertised services
    * - '~reconfigure_inputs' allow to reconfigure the client on the go
    * - '~input_worlds' list the input worlds
    * - '~output_worlds' list the output worlds
    *
    */
  class ReconfigurableClient: public UwdsClientNodelet
  {
  public:
    /**@brief
     * The default constructor
     */
    ReconfigurableClient(): UwdsClientNodelet(UNDEFINED) {}

    /**@brief
     * The default constructor
     */
    ReconfigurableClient(const ClientType& client_type): UwdsClientNodelet(client_type) {}

    /**@brief
     * The default destructor
     */
    ~ReconfigurableClient() = default;

    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit();

  protected:
    /** @brief
     * This method is called when to reconfigure the nodelet.
     *
     * @param new_input_worlds The worlds to connect
     */
    virtual bool reconfigure(const std::vector<std::string>& new_input_worlds);

    /** @brief
     * This method is called when a reconfigure service is received.
     *
     * @param req The reconfigure request
     * @param res The reconfigure response
     */
    virtual bool reconfigureInputs(ReconfigureInputs::Request& req,
                                   ReconfigureInputs::Response& res);

    /** @brief
     * This method is called at the end of the reconfigure process.
     * Add additional initialization in this method.
     *
     * @param new_input_worlds The new input worlds
     */
    virtual void onReconfigure(const std::vector<std::string>& new_input_worlds) = 0;

    /** @brief
     * This method is called when a world changes is subscribed by this nodelet.
     * Set up additional subscribers or class in this method.
     *
     * @param world The world subscribed
     */
    virtual void onSubscribeChanges(const std::string world) = 0;

    /** @brief
     * This method is called when a world changes is unsubscribed by this nodelet.
     * Shut down additional subscribers or class in this method.
     *
     * @param world The world unsubscribed
     */
    virtual void onUnsubscribeChanges(const std::string world) = 0;

   /** @brief
    * This method is called when there is a change in a world,
    * subclass need to implement this method.
    *
    * @param world The world that have been updated
    * @param header The header
    * @param invalidations The invalidations received
    */
    virtual void onChanges(const std::string& world,
                           const std_msgs::Header& header,
                           const Invalidations& invalidations) = 0;


    /** @brief
     * This method is called when changes in worlds are received.
     *
     * @param msgs The messages received
     */
    virtual void applyChanges(const std::vector<ChangesInContextStampedConstPtr>& msgs);

    /** @brief
     * This method is called when a changes in a world are received.
     *
     * @param msg The message received
     */
    virtual void changesCallback(const ChangesInContextStampedConstPtr& msg);

    /** @brief
     * This method is called when a changes in a world are received.
     *
     * @param msg0 The 1st message received
     * @param msg1 The 2nd message received
     */
    virtual void changesCallback(const ChangesInContextStampedConstPtr& msg0,
                                 const ChangesInContextStampedConstPtr& msg1);

    /** @brief
     * This method is called when a changes in worlds are received.
     *
     * @param msg0 The 1st message received
     * @param msg1 The 2nd message received
     * @param msg2 The 3rd message received
     */
    virtual void changesCallback(const ChangesInContextStampedConstPtr& msg0,
                                 const ChangesInContextStampedConstPtr& msg1,
                                 const ChangesInContextStampedConstPtr& msg2);

    /** @brief
     * This method is called when a changes in worlds are received.
     *
     * @param msg0 The 1st message received
     * @param msg1 The 2nd message received
     * @param msg2 The 3rd message received
     * @param msg3 The 4th message received
     */
    virtual void changesCallback(const ChangesInContextStampedConstPtr& msg0,
                                 const ChangesInContextStampedConstPtr& msg1,
                                 const ChangesInContextStampedConstPtr& msg2,
                                 const ChangesInContextStampedConstPtr& msg3);

    /** @brief
     * This method is called when a changes in worlds are received.
     *
     * @param msg0 The 1st message received
     * @param msg1 The 2nd message received
     * @param msg2 The 3rd message received
     * @param msg3 The 4th message received
     * @param msg4 The 5th message received
     */
    virtual void changesCallback(const ChangesInContextStampedConstPtr& msg0,
                                 const ChangesInContextStampedConstPtr& msg1,
                                 const ChangesInContextStampedConstPtr& msg2,
                                 const ChangesInContextStampedConstPtr& msg3,
                                 const ChangesInContextStampedConstPtr& msg4);

    /** @brief
     * This method is called when a changes in worlds are received.
     *
     * @param msg0 The 1st message received
     * @param msg1 The 2nd message received
     * @param msg2 The 3rd message received
     * @param msg3 The 4th message received
     * @param msg4 The 5th message received
     * @param msg5 The 6th message received
     */
    virtual void changesCallback(const ChangesInContextStampedConstPtr& msg0,
                                 const ChangesInContextStampedConstPtr& msg1,
                                 const ChangesInContextStampedConstPtr& msg2,
                                 const ChangesInContextStampedConstPtr& msg3,
                                 const ChangesInContextStampedConstPtr& msg4,
                                 const ChangesInContextStampedConstPtr& msg5);

    /** @brief
     * This method is called when a changes in worlds are received.
     *
     * @param msg0 The 1st message received
     * @param msg1 The 2nd message received
     * @param msg2 The 3rd message received
     * @param msg3 The 4th message received
     * @param msg4 The 5th message received
     * @param msg5 The 6th message received
     * @param msg6 The 7th message received
     */
    virtual void changesCallback(const ChangesInContextStampedConstPtr& msg0,
                                 const ChangesInContextStampedConstPtr& msg1,
                                 const ChangesInContextStampedConstPtr& msg2,
                                 const ChangesInContextStampedConstPtr& msg3,
                                 const ChangesInContextStampedConstPtr& msg4,
                                 const ChangesInContextStampedConstPtr& msg5,
                                 const ChangesInContextStampedConstPtr& msg6);

    /** @brief
     * This method is called when a changes in worlds are received.
     *
     * @param msg0 The 1st message received
     * @param msg1 The 2nd message received
     * @param msg2 The 3rd message received
     * @param msg3 The 4th message received
     * @param msg4 The 5th message received
     * @param msg5 The 6th message received
     * @param msg6 The 7th message received
     * @param msg7 The 8th message received
     */
    virtual void changesCallback(const ChangesInContextStampedConstPtr& msg0,
                                 const ChangesInContextStampedConstPtr& msg1,
                                 const ChangesInContextStampedConstPtr& msg2,
                                 const ChangesInContextStampedConstPtr& msg3,
                                 const ChangesInContextStampedConstPtr& msg4,
                                 const ChangesInContextStampedConstPtr& msg5,
                                 const ChangesInContextStampedConstPtr& msg6,
                                 const ChangesInContextStampedConstPtr& msg7);

    /** @brief
     * This method is called to subscribe to the given world changes.
     *
     * @param world The world to subscribe
     */
    virtual void addChangesSubscriber(const std::string& world);

    /** @brief
     * This method is called to remove the given world changes subscriber.
     *
     * @param world The world to unsubscribe
     */
    virtual void removeChangesSubscriber(const std::string& world);

    /** @brief
     * A flag to know if inputs are syncronized,
     * or not.
     */
    bool synchronized_;

    /** @brief
     * A flag to know if use multiple inputs,
     * or not.
     */
    bool use_multiple_inputs_;

    /** @brief
     * The reconfigure service server.
     */
    ros::ServiceServer reconfigure_service_server_;

    /** @brief
     * The active sync connection.
     */
    message_filters::Connection active_sync_connection_;

    /** @brief
     * The time synchronizer for 2 inputs.
     */
    message_filters::TimeSynchronizer<ChangesInContextStamped, ChangesInContextStamped> * time_synchronizer2;

    /** @brief
     * The time synchronizer for 3 inputs.
     */
    message_filters::TimeSynchronizer<ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped> * time_synchronizer3;

    /** @brief
     * The time synchronizer for 4 inputs.
     */
    message_filters::TimeSynchronizer<ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped> * time_synchronizer4;

    /** @brief
     * The time synchronizer for 5 inputs.
     */
    message_filters::TimeSynchronizer<ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped> * time_synchronizer5;

    /** @brief
     * The time synchronizer for 6 inputs.
     */
    message_filters::TimeSynchronizer<ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped> * time_synchronizer6;

    /** @brief
     * The time synchronizer for 7 inputs.
     */
    message_filters::TimeSynchronizer<ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped> * time_synchronizer7;

    /** @brief
     * The time synchronizer for 8 inputs.
     */
    message_filters::TimeSynchronizer<ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped, ChangesInContextStamped> * time_synchronizer8;

  };
}

#endif
