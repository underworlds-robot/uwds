#ifndef UWDS_SERVER_HPP
#define UWDS_SERVER_HPP

#include "uwds/uwds.h"
#include <nodelet/nodelet.h>

namespace uwds
{
  class UwdsServer : public nodelet::Nodelet
  {
  public:
    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit();

  private:
    /**
     * The ROS node handle shared pointer.
     */
    boost::shared_ptr<ros::NodeHandle> nh_;
    /** @brief
     * The Underworlds server.
     */
    boost::shared_ptr<Underworlds> ctx_;
  };

}

#endif
