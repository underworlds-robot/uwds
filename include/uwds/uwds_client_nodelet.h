#ifndef UWDS_CLIENT_NODELET_HPP
#define UWDS_CLIENT_NODELET_HPP

#include "uwds/uwds.h"
#include <nodelet/nodelet.h>

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds
{
  class UwdsClientNodelet : public nodelet::Nodelet
  {
  public:
    UwdsClientNodelet(ClientType type) : type_(type) {}
    ~UwdsClientNodelet() = default;
    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit();

  protected:
    /**
     * The ROS node handle shared pointer.
     */
    boost::shared_ptr<ros::NodeHandle> nh_;

    /**
     * The ROS private node handle shared pointer.
     */
    boost::shared_ptr<ros::NodeHandle> pnh_;

    /** @brief
     * The Underworlds client proxy.
     */
    boost::shared_ptr<UnderworldsProxy> ctx_;

    ClientType type_;

    bool verbose_;

    string global_frame_id_;
  };
}

#endif
