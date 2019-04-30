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
      node_id_map_[node.name] = node;
    }
  }

  void RobotProvider::callback(const boost::shared_ptr<JointState>& msg)
  {
    Changes changes;
    Node node_to_update;

    for (uint i = 0; i < msg->name.size(); ++i)
    {
      node_to_update = node_id_map_[msg->name[i]];
      vector<float> axis;
      JointType type;
      for(const auto& property : node_to_update.properties)
      {
        if(property.name == "axis")
        {
          vector<string> axis_tokens;
          boost::split(property.data, axis_tokens, boost::is_any_of(","));
          for (size_t j = 0; j <3 ; ++j) {
            axis.push_back(to_float(axis_tokens[j]));
          }
        }
        if(property.name == "joint")
        {
          switch (property.data) {
            case JointTypeName[REVOLUTE]: type = REVOLUTE; break;
            case JointTypeName[CONTINUOUS]: type = CONTINUOUS; break;
            case JointTypeName[PRISMATIC]: type = PRISMATIC; break;
            case JointTypeName[FLOATING]: type = FLOATING; break;
            case JointTypeName[PLANAR]: type = PLANAR; break;
            case JointTypeName[FIXED]: type = FIXED; break;
            default : NODELET_ERROR("[%s::callback] Unknown type for joint %s, please check the robot URDF", ctx_->name().c_str(), msg->name[i]);
                      return;
          }
        }
        if (type != FIXED)
        {
          
        }
        //TODO apply transform
      }
    }



  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds::RobotProvider, nodelet::Nodelet)
