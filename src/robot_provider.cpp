#include "uwds/env_provider.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds
{
  void RobotProvider::onInit()
  {
    UwdsClientNodelet::onInit();
    string filename, primitives_folder;
    pnh_->param<string>("filename", filename, "");
    pnh_->param<string>("primitives_folder", primitives_folder, "");
    pnh_->param<string>("base_frame_id", base_frame_id_, "base_link");
    vector<Node> nodes_imported;
    if(ctx_->worlds()[output_world_].pushRobotMeshesFromURDF(filename, primitives_folder, nodes_imported))
      if(verbose_)ROS_INFO("Successfully load file !");
    for(const auto node : nodes_imported)
    {
      link_id_map_[node.name] = node;
    }
  }

  void RobotProvider::callback(const boost::shared_ptr<JointState>& msg)
  {
    Changes changes;
    // for (uint i = 0; i < msg->name.size(); ++i)
    // {
    //   node_to_update = link_id_map_[msg->name[i]];
    //   //node_to_update.position.pose.position.x = msg->position[i]
    //
    // }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds::RobotProvider, nodelet::Nodelet)
