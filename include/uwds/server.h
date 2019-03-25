#ifndef UWDS_SERVER_HPP
#define UWDS_SERVER_HPP

#include "uwds/uwds.h"
#include <nodelet/nodelet.h>

namespace uwds
{
  class UwdsServerNodelet : public nodelet::Nodelet
  {
  public:
    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit();

  private:

    bool verbose_ = false;
    /**
     * The ROS node handle shared pointer.
     */
    boost::shared_ptr<ros::NodeHandle> nh_;

    /**
     * The ROS privqte node handle shared pointer.
     */
    boost::shared_ptr<ros::NodeHandle> pnh_;
    /** @brief
     * The Underworlds server.
     */
    boost::shared_ptr<Underworlds> ctx_;
  };

}

#endif
