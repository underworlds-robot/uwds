#include "uwds/simple_object_provider.h"

namespace uwds
{
  void SimpleObjectProvider::onInit()
  {
    UwdsClientNodelet::onInit();
    pnh_->param<string>("output_world", output_world_, "simple_object");
    pnh_->param<string>("global_frame_id", global_frame_id_, "map");
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
      Changes changes;

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

        ros::Time now = ros::Time::now();
        double deltaX = t.getX() - object_node_->position.pose.position.x;
        double deltaY = t.getY() - object_node_->position.pose.position.y;
        double deltaZ = t.getZ() - object_node_->position.pose.position.z;
        double deltaT;

        if (object_node_->last_observation.data.toSec()!= 0.0)
        {
          if(now.toSec() <= object_node_->last_observation.data.toSec())
            return;
          deltaT = now.toSec() - object_node_->last_observation.data.toSec();
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

        object_node_->position.pose.position.x = t.getX();
        object_node_->position.pose.position.y = t.getY();
        object_node_->position.pose.position.z = t.getZ();

        object_node_->position.pose.orientation.x = 0.0;
        object_node_->position.pose.orientation.y = 0.0;
        object_node_->position.pose.orientation.z = 0.0;
        object_node_->position.pose.orientation.w = 1.0;

        object_node_->last_observation.data = now;

        if (!use_mesh_)
        {
          for (auto& property : object_node_->properties)
          {
            if (property.name == "aabb")
              property.data = to_string(msg->dimensions.x) + "," + to_string(msg->dimensions.y) + "," + to_string(msg->dimensions.z);
          }
        }
        changes.nodes_to_update.push_back(*object_node_);
        Header header;
        header.frame_id = global_frame_id_;
        header.stamp = msg->header.stamp;
        ctx_->worlds()[output_world_].update(header, changes);
      }
      catch (tf::TransformException &ex) {
        NODELET_WARN("[%s] Exception occured : %s",ctx_->name().c_str(), ex.what());
      }
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds::SimpleObjectProvider, nodelet::Nodelet)
