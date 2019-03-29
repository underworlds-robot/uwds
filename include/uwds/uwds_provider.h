#ifndef UWDS_PROVIDER_HPP
#define UWDS_PROVIDER_HPP

#include "uwds/uwds_client_nodelet.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds
{
  template<typename Message>
  class UwdsProvider : public UwdsClientNodelet
  {
  public:

    UwdsProvider<Message>():UwdsClientNodelet(PROVIDER) {}
    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit();

  private:

    /**@brief
     * This method is called when data are received.
     */
    virtual void callback(const boost::shared_ptr<Message>& msg) = 0;

    /**@brief
     * Input subscriber for perception data.
     */
    ros::Subscriber input_subscriber_;
  };
}

#endif
