#include "uwds/simple_object_provider.h"

namespace uwds
{
  void SimpleObjectProvider::onInit()
  {
    UwdsClientNodelet::onInit();
    pnh_->param<string>("output_world", output_world_, "simple_object");
    pnh_->param<string>("global_frame_id", global_frame_id_, "map");
    pnh_->param<bool>("align_with_world", align_with_world_, true);
    pnh_->param<bool>("use_mesh", use_mesh_, true);
    object_node_ = boost::make_shared<Node>();
    Property meshes_property;
    meshes_property.name = "meshes";
    Property aabb_property;
    aabb_property.name = "aabb";
    Property class_property;
    class_property.name = "class";

    pnh_->param<string>("object_name", object_name_, "unknown");
    pnh_->param<string>("object_class", object_class_, "Artifact");
    class_property.data = object_class_;
    string object_model;
    pnh_->param<string>("object_model", object_model, "");

    vector<double> aabb;
    if (ctx_->worlds()[output_world_].pushMeshesFrom3DFile(object_model, object_meshes_, aabb))
    {
      use_mesh_ = true;
      for (unsigned int i=0 ; i < object_meshes_.size() ; i++)
      {
        meshes_property.data += object_meshes_[i].id;
        if (i < object_meshes_.size()-1)
        {
          meshes_property.data += ",";
        }
      }
      if (aabb.size() == 3)
        aabb_property.data = to_string(aabb[0]) + "," + to_string(aabb[1]) + "," + to_string(aabb[2]);
    } else {
      NODELET_ERROR("[%s] Error while loading file '%s'", ctx_->name().c_str(), object_model.c_str());
      return;
    }
    object_node_->id = NEW_UUID;
    object_node_->name = object_name_;
    object_node_->type = MESH;
    object_node_->properties.push_back(meshes_property);
    object_node_->properties.push_back(aabb_property);
    object_node_->properties.push_back(class_property);
    input_subscriber_ = pnh_->subscribe("input", 1, &SimpleObjectProvider::callback, this);
  }

  void SimpleObjectProvider::callback(const BoundingBoxConstPtr& msg)
  {
    if (msg->pose.position.x!=0 && msg->pose.position.y!=0 && msg->pose.position.z !=0)
    {
      bool transformed = false;
      Header header;
      header.frame_id = msg->header.frame_id;
      header.stamp = ros::Time::now();
      Changes changes;
      float tx = msg->pose.position.x;
      float ty = msg->pose.position.y;
      float tz = msg->pose.position.z;

      float rx = msg->pose.orientation.x;
      float ry = msg->pose.orientation.y;
      float rz = msg->pose.orientation.z;
      float rw = msg->pose.orientation.w;

      if(align_with_world_)
      {
        static tf::TransformBroadcaster br;
        tf::Transform box_tr;
        box_tr.setOrigin(tf::Vector3(msg->pose.position.x,
                                        msg->pose.position.y,
                                        msg->pose.position.z));
        box_tr.setRotation(tf::Quaternion(msg->pose.orientation.x,
                                             msg->pose.orientation.y,
                                             msg->pose.orientation.z,
                                             msg->pose.orientation.w));
        br.sendTransform(tf::StampedTransform(box_tr, msg->header.stamp, msg->header.frame_id, object_name_));

        try
        {
          tf::StampedTransform transform;
          tf_listener_.lookupTransform(global_frame_id_, object_name_,
                                   ros::Time(), transform);
          tf::Vector3 t = transform.getOrigin();
          tf::Quaternion q = transform.getRotation();
          tx = t.getX();
          ty = t.getY();
          tz = t.getZ();
          rx = 0.0;
          ry = 0.0;
          rz = 0.0;
          rw = 1.0;
          transformed = true;
        } catch (tf::TransformException &ex) {
          NODELET_WARN("[%s] Exception occured : %s",ctx_->name().c_str(), ex.what());
        }
      }


      double deltaX = tx - object_node_->position.pose.position.x;
      double deltaY = ty - object_node_->position.pose.position.y;
      double deltaZ = tz - object_node_->position.pose.position.z;
      double deltaT;

      if (object_node_->last_observation.data.toSec()!= 0.0)
      {
        if(header.stamp.toSec() <= object_node_->last_observation.data.toSec())
          return;
        deltaT = header.stamp.toSec() - object_node_->last_observation.data.toSec();
      }
      else {
        deltaT = 0.0;
      }

      if(deltaT != 0.0)
      {
        if(deltaX != 0.0)
        {
          object_node_->velocity.twist.linear.x = deltaX / deltaT;
        } else {
          object_node_->velocity.twist.linear.x = 0.0;
        }
        if(deltaY != 0.0)
        {
          object_node_->velocity.twist.linear.y = deltaY / deltaT;
        } else {
          object_node_->velocity.twist.linear.y = 0.0;
        }
        if(deltaZ != 0.0)
        {
          object_node_->velocity.twist.linear.z = deltaZ / deltaT;
        } else {
          object_node_->velocity.twist.linear.z = 0.0;
        }
      } else {
        object_node_->velocity.twist.linear.x = NAN;
        object_node_->velocity.twist.linear.y = NAN;
        object_node_->velocity.twist.linear.z = NAN;
      }

      object_node_->position.pose.position.x = tx;
      object_node_->position.pose.position.y = ty;
      object_node_->position.pose.position.z = tz;

      object_node_->position.pose.orientation.x = rx;
      object_node_->position.pose.orientation.y = ry;
      object_node_->position.pose.orientation.z = rz;
      object_node_->position.pose.orientation.w = rw;

      object_node_->last_observation.data = header.stamp;

      if (!use_mesh_)
      {
        for (auto& property : object_node_->properties)
        {
          if (property.name == "aabb")
            property.data = to_string(msg->dimensions.x) + "," + to_string(msg->dimensions.y) + "," + to_string(msg->dimensions.z);
        }
      }

      changes.nodes_to_update.push_back(*object_node_);
      if(transformed == true)
        header.frame_id = global_frame_id_;
      ctx_->worlds()[output_world_].update(header, changes);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds::SimpleObjectProvider, nodelet::Nodelet)
