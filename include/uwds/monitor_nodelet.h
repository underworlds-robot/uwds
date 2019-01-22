#ifndef MONITOR_NODELET_HPP
#define MONITOR_NODELET_HPP

#include "uwds/uwds.h"
#include "uwds/reconfigurable_client.h"

using namespace uwds_msgs;
using namespace uwds;

namespace uwds
{
  /** @brief
   * This asbstract class allow to help to design monitors.
   *
   * Monitors are processes that analyse the geometry and manage situations
   * of a world.
   */
  class MonitorNodelet : public ReconfigurableClient
  {
    public:
      /** @brief
       * The default constructor.
       */
      MonitorNodelet(): ReconfigurableClient(PROVIDER) {}

      /**@brief
       * The default destructor.
       */
      ~MonitorNodelet() = default;

      /** @brief
       * Initialize method. Subclass should call this method
       * in its onInit method.
       */
      virtual void onInit()
      {
        ReconfigurableClient::onInit();
      }

      /** @brief
       * The monitor method to implement in subclass.
       */
      virtual Changes monitor(const Invalidations& input_invalidations) = 0;

    private:
      /**@brief
       * This method is called when data are received.
       */
      virtual void callback(const boost::shared_ptr<RosMessage>& msg) = 0;

      /** @brief
       * The output world name.
       */
      std::string output_world_;

      /**@brief
       * Input subscriber for perception data.
       */
      ros::Subscriber input_subscriber_;
  };
}

#endif
