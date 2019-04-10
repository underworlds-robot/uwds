#include "uwds/world_merger.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds
{
  void WorldMerger::onInit()
  {
    tf_buffer_ = boost::make_shared<tf2_ros::Buffer>();
    tf_listener_ = boost::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    ReconfigurableClient::onInit();
    float publisher_frequency;
    pnh_->param<float>("publisher_frequency", publisher_frequency, 30.0);
    publisher_timer_ = nh_->createTimer(ros::Duration(1.0/publisher_frequency), &WorldMerger::onTimer, this);
  }

  void WorldMerger::onChanges(const string& world,
                              const Header& header,
                              const Invalidations& invalidations)
  {
    auto& scene = ctx_->worlds()[world].scene();
    auto& timeline = ctx_->worlds()[world].timeline();
    auto& meshes = ctx_->worlds()[world].meshes();

    changes_mutex_.lock();
    for(const string& id : invalidations.node_ids_updated)
    {
      if(scene.nodes().has(id)) {
        if (scene.nodes()[id].name != "root")
        {
          bool insert = false;
          bool transformed = false;
          if(header.frame_id!="" && header.frame_id != global_frame_id_)
          {
            vector<Node> match = scene.getNodesByName(header.frame_id);
            if(match.size() > 0)
            {
              if(match.size() > 1)
              {
                NODELET_WARN("[%s::onChanges] Multiple nodes matching '%s' frame. Parenting to the first one.", ctx_->name().c_str(), header.frame_id.c_str());
              }
              scene.nodes()[id].parent = match[0].id;
              transformed = true;
            } else {
              geometry_msgs::TransformStamped transformStamped;
              tf2::Stamped<tf2::Transform> temp;
              geometry_msgs::PoseStamped sensor_pose;
              try {
                transformStamped = tf_buffer_->lookupTransform(header.frame_id, global_frame_id_, ros::Time(0));
                tf2::fromMsg(transformStamped, temp);
                tf2::toMsg(temp, sensor_pose);
                pose_cov_ops::inverseCompose(scene.nodes()[id].position.pose, sensor_pose.pose, scene.nodes()[id].position.pose);
                transformed = true;
              } catch(tf2::TransformException& e) {
                ROS_WARN("[%s::onChanges] Error occured while transforming node into world frame : %s",ctx_->name().c_str(), e.what());
              }
            }
          } else {
            transformed = true;
          }
          if (transformed)
          {
            for (uint i=0; i < changes_to_send_.nodes_to_update.size(); ++i) {
              if (changes_to_send_.nodes_to_update[i].id == id)
              {
                changes_to_send_.nodes_to_update[i] = scene.nodes()[id];
                insert = true;
              }
            }
            if(!insert) changes_to_send_.nodes_to_update.push_back(scene.nodes()[id]);
          }
        }
      }
    }
    for(const string& id : invalidations.situation_ids_updated)
    {
      bool insert = false;
      for (uint i=0; i < changes_to_send_.situations_to_update.size(); ++i) {
        if (changes_to_send_.situations_to_update[i].id == id)
        {
          if(timeline.situations().has(id)) {
            changes_to_send_.situations_to_update[i] = timeline.situations()[id];
            insert = true;
          }
        }
      }
      if(timeline.situations().has(id))
        if(!insert) changes_to_send_.situations_to_update.push_back(timeline.situations()[id]);
    }
    for(const std::string& id : invalidations.mesh_ids_updated)
    {
      bool insert = false;
      for (uint i=0; i < changes_to_send_.meshes_to_update.size(); ++i) {
        if (changes_to_send_.meshes_to_update[i].id == id)
        {
          if(meshes.has(id)) {
            changes_to_send_.meshes_to_update[i] = meshes[id];
            insert = true;
          }
        }
      }
      if(meshes.has(id))
        if(!insert) changes_to_send_.meshes_to_update.push_back(meshes[id]);
    }

    for(const string& id : invalidations.node_ids_deleted)
    {
      bool insert = false;
      for (uint i=0; i < changes_to_send_.nodes_to_delete.size(); ++i)
      {
        if (changes_to_send_.nodes_to_delete[i] == id)
          insert = true;
      }
      if(!insert) changes_to_send_.nodes_to_delete.push_back(id);
    }

    for(const string& id : invalidations.situation_ids_deleted)
    {
      bool insert = false;
      for (uint i=0; i < changes_to_send_.situations_to_delete.size(); ++i)
      {
        if (changes_to_send_.situations_to_delete[i] == id)
          insert = true;
      }
      if(!insert) changes_to_send_.situations_to_delete.push_back(id);
    }

    for(const string& id : invalidations.mesh_ids_deleted)
    {
      bool insert = false;
      for (uint i=0; i < changes_to_send_.meshes_to_delete.size(); ++i)
      {
        if (changes_to_send_.meshes_to_delete[i] == id)
          insert = true;
      }
      if(!insert) changes_to_send_.meshes_to_delete.push_back(id);
    }
    changes_mutex_.unlock();
  }

  void WorldMerger::onTimer(const ros::TimerEvent& event)
  {
    for (const auto& world : input_worlds_)
    {
      Header header;
      header.stamp = ros::Time::now();
      header.frame_id = global_frame_id_;
      changes_mutex_.lock();
      ctx_->worlds()[output_world_].update(header, changes_to_send_);
      changes_to_send_.nodes_to_update.clear();
      changes_to_send_.situations_to_update.clear();
      changes_to_send_.meshes_to_update.clear();
      changes_to_send_.nodes_to_delete.clear();
      changes_to_send_.situations_to_delete.clear();
      changes_to_send_.meshes_to_delete.clear();
      changes_mutex_.unlock();
      if(verbose_)NODELET_INFO("[%s::onChanges] Send changes to world <%s>", ctx_->name().c_str(), output_world_.c_str());
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds::WorldMerger, nodelet::Nodelet)
