#include "uwds/env_provider.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds
{
  void EnvProvider::onInit()
  {
    UwdsClientNodelet::onInit();
    string filename;

    pnh_->param<string>("filename", filename, "");
    if(ctx_->worlds()[output_world_].pushSceneFrom3DFile(filename))
      if(verbose_)ROS_INFO("Successfully load file !");
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds::EnvProvider, nodelet::Nodelet)
