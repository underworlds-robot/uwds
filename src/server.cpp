#include "uwds/server.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds
{
  void UwdsServerNodelet::onInit()
  {
    nh_ = boost::make_shared<ros::NodeHandle>(getMTNodeHandle());
    ctx_ = boost::make_shared<Underworlds>(nh_);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds::UwdsServerNodelet, nodelet::Nodelet)
