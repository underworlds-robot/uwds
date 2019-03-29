#include "uwds/uwds_client_nodelet.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds
{
  void UwdsClientNodelet::onInit()
  {
    nh_ = boost::make_shared<ros::NodeHandle>(getMTNodeHandle());
    pnh_ = boost::make_shared<ros::NodeHandle>(getMTPrivateNodeHandle());
    string name = getName();
    name.erase(std::begin(name));
    ctx_ = boost::make_shared<UnderworldsProxy>(nh_, pnh_ , name, type_);
    pnh_->param<bool>("verbose", verbose_, false);
    pnh_->param<string>("global_frame_id", global_frame_id_, "odom");
    pnh_->param<string>("output_world", output_world_, "");
    pnh_->param<string>("output_suffix", output_suffix_, "");
  }
}
