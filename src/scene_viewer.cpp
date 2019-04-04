#include "uwds/scene_viewer.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;
using namespace visualization_msgs;
using namespace jsk_recognition_msgs;

namespace uwds
{
  void SceneViewer::onInit()
  {
    uwds::ReconfigurableClient::onInit();

    float publisher_frequency;
    pnh_->param<float>("publisher_frequency", publisher_frequency, 20.0);
    publisher_timer_ = nh_->createTimer(ros::Duration(1.0/publisher_frequency), &SceneViewer::onTimer, this);
  }

  void SceneViewer::onReconfigure(const vector<string>& inputs)
  {
    markers_publisher_map_.clear();
    bboxes_publisher_map_.clear();
    camera_publishers_map_.clear();
    marker_id_map_.clear();
    last_marker_id_ = 0;

    for(const auto& input : inputs)
    {
      markers_publisher_map_.emplace(input, boost::make_shared<ros::Publisher>(nh_->advertise<MarkerArray>(input+"/meshes", 2)));
      bboxes_publisher_map_.emplace(input, boost::make_shared<ros::Publisher>(nh_->advertise<BoundingBoxArray>(input+"/boxes", 2)));
    }
  }

  void SceneViewer::publishVisualization(const std::string world, const ros::Time stamp)
  {
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = global_frame_id_;

    BoundingBoxArray bboxes;
    MarkerArray markers;
    bboxes.header = header;
    auto& scene = ctx_->worlds()[world].scene();
    scene.lock();
    for (const auto& node_ptr : scene.nodes())
    {
      Node node = *node_ptr;
      if (node.name != "root")
      {
        std::string source_frame;
        if (node.parent != scene.rootID())
        {
          source_frame = world + "/" + scene.nodes()[node.parent].id;
        } else {
          source_frame = global_frame_id_;
        }
        tf::Transform transform;

        tf::Vector3 t(node.position.pose.position.x,
                      node.position.pose.position.y,
                      node.position.pose.position.z);
        tf::Quaternion q(node.position.pose.orientation.x,
                         node.position.pose.orientation.y,
                         node.position.pose.orientation.z,
                         node.position.pose.orientation.w);
        q.normalize();
        transform.setOrigin(t);
        transform.setRotation(q);

        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, stamp, source_frame, world + "/" + node.id));

        if (node.type == MESH)
        {
          for(auto marker : nodeToMarkers(world, node, stamp))
          {
            markers.markers.push_back(marker);
          }
          bboxes.boxes.push_back(nodeToBoundingBox(world, node, stamp));
        }

        if (node.type == CAMERA)
        {
          if(camera_publishers_map_.count(world+node.id) == 0)
            camera_publishers_map_.emplace(world+node.id, boost::make_shared<ros::Publisher>(nh_->advertise<sensor_msgs::CameraInfo>(world+"/"+node.id+"/camera_info", 2)));

          camera_publishers_map_.at(world+node.id)->publish(nodeToCameraInfo(world, node, stamp));
        }
      }
    }
    scene.unlock();
    if (markers_publisher_map_.count(world) > 0  && markers.markers.size() > 0)
      markers_publisher_map_.at(world)->publish(markers);

    if (bboxes_publisher_map_.count(world) > 0 && bboxes.boxes.size() > 0)
      bboxes_publisher_map_.at(world)->publish(bboxes);
  }

  void SceneViewer::onTimer(const ros::TimerEvent& event) {
    try{
      for (const auto& world : input_worlds_)
      {
        ros::Time stamp = ros::Time::now();
        publishVisualization(world, stamp);
      }
    } catch(std::exception e) {

    }
  }

  vector<Marker> SceneViewer::nodeToMarkers(const string world, const Node node, const ros::Time stamp)
  {
    vector<Marker> markers;
    vector<string> mesh_ids;

    for(auto property : node.properties)
    {
      if(property.name=="meshes")
      {
        boost::split(mesh_ids, property.data, boost::is_any_of(","), boost::token_compress_on);
        break;
      }
    }

    auto& scene = ctx_->worlds()[world].scene();

    for(auto mesh_id : mesh_ids)
    {
      Marker marker;
      if (node.parent == scene.rootID())
        marker.header.frame_id = global_frame_id_;
      else
        marker.header.frame_id = world + "/" + scene.nodes()[node.parent].id;
      marker.header.stamp = stamp;
      if (marker_id_map_.count(world+mesh_id)==0) {
        marker_id_map_.emplace(world+mesh_id, last_marker_id_++);
      }
      marker.id = marker_id_map_.at(world+mesh_id);
      marker.action = Marker::ADD;
      marker.type = Marker::TRIANGLE_LIST;
      marker.pose = node.position.pose;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      const auto& mesh = ctx_->meshes()[mesh_id];
      for (const auto& triangle : mesh.triangles)
      {
        geometry_msgs::Point p0, p1, p2;
        std_msgs::ColorRGBA c0, c1, c2;
        p0.x = mesh.vertices[triangle.vertex_indices[0]].x;
        p0.y = mesh.vertices[triangle.vertex_indices[0]].y;
        p0.z = mesh.vertices[triangle.vertex_indices[0]].z;

        c0.r = mesh.vertex_colors[triangle.vertex_indices[0]].r;
        c0.g = mesh.vertex_colors[triangle.vertex_indices[0]].g;
        c0.b = mesh.vertex_colors[triangle.vertex_indices[0]].b;
        c0.a = mesh.vertex_colors[triangle.vertex_indices[0]].a;

        p1.x = mesh.vertices[triangle.vertex_indices[1]].x;
        p1.y = mesh.vertices[triangle.vertex_indices[1]].y;
        p1.z = mesh.vertices[triangle.vertex_indices[1]].z;

        c1.r = mesh.vertex_colors[triangle.vertex_indices[1]].r;
        c1.g = mesh.vertex_colors[triangle.vertex_indices[1]].g;
        c1.b = mesh.vertex_colors[triangle.vertex_indices[1]].b;
        c1.a = mesh.vertex_colors[triangle.vertex_indices[1]].a;

        p2.x = mesh.vertices[triangle.vertex_indices[2]].x;
        p2.y = mesh.vertices[triangle.vertex_indices[2]].y;
        p2.z = mesh.vertices[triangle.vertex_indices[2]].z;

        c2.r = mesh.vertex_colors[triangle.vertex_indices[2]].r;
        c2.g = mesh.vertex_colors[triangle.vertex_indices[2]].g;
        c2.b = mesh.vertex_colors[triangle.vertex_indices[2]].b;
        c2.a = mesh.vertex_colors[triangle.vertex_indices[2]].a;

        marker.points.push_back(p0);
        marker.colors.push_back(c0);

        marker.points.push_back(p1);
        marker.colors.push_back(c1);

        marker.points.push_back(p2);
        marker.colors.push_back(c2);
      }
      markers.push_back(marker);
    }
    return markers;
  }

  BoundingBox SceneViewer::nodeToBoundingBox(const string world, const Node node, const ros::Time stamp)
  {
    BoundingBox bbox;
    std_msgs::Header bbox_header;
    bbox.header.stamp = stamp;
    bbox.header.frame_id = global_frame_id_;
    try
    {
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(global_frame_id_, world+"/"+node.id,
                               ros::Time(), transform);

      tf::Vector3 t = transform.getOrigin();
      tf::Quaternion q = transform.getRotation();
      bbox.pose.position.x = t.getX();
      bbox.pose.position.y = t.getY();
      bbox.pose.position.z = t.getZ();

      bbox.pose.orientation.x = q.getX();
      bbox.pose.orientation.y = q.getY();
      bbox.pose.orientation.z = q.getZ();
      bbox.pose.orientation.w = q.getW();
    } catch(exception){
      bbox.header.frame_id = world+"/"+node.name;
    }
    for(const auto& property : node.properties)
    {
      if (property.name == "aabb")
      {
        vector<string> aabb_data;
        boost::split(aabb_data, property.data, boost::is_any_of(","), boost::token_compress_on);
        if (aabb_data.size() == 3)
        {
          bbox.dimensions.x = atof(aabb_data[0].c_str());
          bbox.dimensions.y = atof(aabb_data[1].c_str());
          bbox.dimensions.z = atof(aabb_data[2].c_str());
        }
      }
    }
    return bbox;
  }

  sensor_msgs::CameraInfo SceneViewer::nodeToCameraInfo(const string world, const Node node, const ros::Time stamp)
  {
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = world + "/" + node.id;
    camera_info.header.stamp = stamp;
    camera_info.height=480;
    camera_info.width=640;
    camera_info.distortion_model="plumb_bob";
    camera_info.D.push_back(0.007313);
    camera_info.D.push_back(-0.068372);
    camera_info.D.push_back(-0.002248);
    camera_info.D.push_back(0.001679);
    camera_info.D.push_back(0.0);
    camera_info.K[0] = 509.015381;
    camera_info.K[0] = 0.0;
    camera_info.K[0] = 320.675346;
    camera_info.K[0] = 0.0;
    camera_info.K[0] = 508.683267;
    camera_info.K[0] = 236.130072;
    camera_info.K[0] = 0.0;
    camera_info.K[0] = 0.0;
    camera_info.K[0] = 1.0;

    camera_info.R[0] = 0.999627;
    camera_info.R[1] = 0.007759;
    camera_info.R[2] = 0.026199;
    camera_info.R[3] = -0.007718;
    camera_info.R[4] = 0.999969;
    camera_info.R[5] = -0.001661;
    camera_info.R[6] = -0.026211;
    camera_info.R[7] = 0.001459;
    camera_info.R[8] = 0.999655;

    camera_info.P[0] = 514.03069;
    camera_info.P[1] = 0.0;
    camera_info.P[2] = 302.063255;
    camera_info.P[3] = 0.0;
    camera_info.P[4] = 0.0;
    camera_info.P[5] = 514.03069;
    camera_info.P[6] = 236.707602;
    camera_info.P[7] = 0.0;
    camera_info.P[8] = 0.0;
    camera_info.P[9] = 0.0;
    camera_info.P[10] = 1.0;
    camera_info.P[11] = 0.0;
    return camera_info;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds::SceneViewer, nodelet::Nodelet)
