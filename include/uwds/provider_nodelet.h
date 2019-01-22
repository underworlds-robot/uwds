#ifndef PROVIDER_NODELET_HPP
#define PROVIDER_NODELET_HPP

#include "uwds/uwds_client_nodelet.h"

using namespace uwds_msgs;

namespace uwds
{
  /** @brief
   * This asbstract class allow to help to design providers.
   *
   * Providers are processes that provide data from others systems, like oracles.
   * They don't read into Underworlds worlds so they don't need to be reconfigurable.
   */
  template<struct Message>
  class ProviderNodelet : public uwds::UwdsClientNodelet
  {
    public:
      /** @brief
       * The default constructor.
       */
      ProviderNodelet(): uwds::UwdsClientNodelet(uwds::PROVIDER) {}

      /**@brief
       * The default destructor.
       */
      ~ProviderNodelet() = default;

      /** @brief
       * Initialize method. Subclass should call this method
       * in its onInit method.
       */
      virtual void onInit()
      {
        uwds::UwdsClientNodelet::onInit();
        input_subscriber_ = pnh_->subscribe("input", 1, &ProviderNodelet::callback, this);
        connection_status_ = CONNECTED;
      }

    private:
      /**@brief
       * This method is called when data are received.
       */
      virtual void callback(const boost::shared_ptr<Message>& msg) = 0;

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
