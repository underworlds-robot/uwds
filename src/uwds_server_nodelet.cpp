#include "uwds/uwds_server_nodelet.h"

using namespace std;
using namespace uwds_msgs;

namespace uwds
{
  void UwdsServerNodelet::onInit()
  {
    DynamicConnectionBasedNodelet::onInit();
    // Changes subscriber (for zopy-copy pointer passing)
    // Used to subscribe to the changes
    changes_subscriber_ = boost::make_shared<ros::Subscriber>(nh_->subscribe("uwds/changes",
                                                              subscriber_buffer_size_,
                                                              &UwdsServerNodelet::changesCallback,
                                                              this));
    // General service
    // Service called to get clients topology
    get_topology_service_server_ = nh_->advertiseService("uwds/get_topology",
                                                         &UwdsServerNodelet::getTopology,
                                                         this);
    if(verbose_)NODELET_INFO("[%s] Service server 'uwds/get_topology' created", nodelet_name_.c_str());
  	// Nodes related service
    // Service called to load the scene
    get_scene_service_server_ = nh_->advertiseService("uwds/get_scene",
                                                      &UwdsServerNodelet::getScene,
                                                      this);
    if(verbose_)NODELET_INFO("[%s] Service server 'uwds/get_scene' created", nodelet_name_.c_str());
    // Situations related service
    // Service called to load previous situations
    get_timeline_service_server_ = nh_->advertiseService("uwds/get_timeline",
                                                         &UwdsServerNodelet::getTimeline,
                                                         this);
    if(verbose_)NODELET_INFO("[%s] Service server 'uwds/get_timeline' created", nodelet_name_.c_str());
    // Meshes related service
    // Service called to load the meshes
    get_mesh_service_server_ = nh_->advertiseService("uwds/get_mesh",
                                      &UwdsServerNodelet::getMesh,
                                      this);
    if(verbose_)NODELET_INFO("[%s] Service server 'uwds/get_mesh' created",
                    nodelet_name_.c_str());

    // Timer to clean the situations ended over buffer size
   float clean_up_timer_duration;
   pnh_->param<float>("clean_up_timer_duration",
                      clean_up_timer_duration,
                      DEFAULT_CLEAN_UP_TIMER_DURATION);
    situations_clean_up_timer_ = nh_->createTimer(ros::Duration(clean_up_timer_duration),
                                                          &UwdsServerNodelet::cleanUpTimerCallback,
                                                          this);
    // Server ready
    NODELET_INFO("[%s] Server ready !", nodelet_name_.c_str());
    connection_status_ = CONNECTED;
  }

  bool UwdsServerNodelet::getTopology(uwds_msgs::GetTopology::Request &req,
                   				uwds_msgs::GetTopology::Response &res)
  {
  	NODELET_INFO("[%s] Request uwds/get_topology", nodelet_name_.c_str());
  	try {
      for (auto world : topology().worlds())
      {
        res.worlds.push_back(world);
      }
      for (auto client : topology().clients())
      {
        res.clients.push_back(*client);
      }
      for (auto interactions : topology().client_interactions())
      {
        for (auto interaction : *interactions) {
          res.client_interactions.push_back(interaction);
        }
      }
  		res.success = true;
  	} catch(const std::exception& e) {
      	NODELET_ERROR("[%s] Exception occured : %s", nodelet_name_.c_str(), e.what());
      	res.success = false;
      	res.error = e.what();
    }
    return true;
  }

  bool UwdsServerNodelet::getScene(uwds_msgs::GetScene::Request &req,
                                   uwds_msgs::GetScene::Response &res)
  {
  	if(verbose_)NODELET_INFO("[%s] Client <%s> request uwds/get_scene in <%s> world", nodelet_name_.c_str(), req.ctxt.client.name.c_str(), req.ctxt.world.c_str());
    try
    {
      if (req.ctxt.client.id != "")
        topology().update(req.ctxt, READ);
    	addChangesPublisher(req.ctxt.world);
    	auto& scene = worlds()[req.ctxt.world].scene();
      uint i=0;
      NODELET_INFO("[%s] %d nodes in the scene", nodelet_name_.c_str(), (uint)scene.nodes().size());
    	for (auto node : scene.nodes())
    	{
    		res.nodes.push_back(*node);
    	}
    	res.root_id = scene.rootID();
    	res.success = true;
    	res.error = "";
    }
    catch(const std::exception& e)
    {
    	NODELET_ERROR("[%s] Exception occured : %s", nodelet_name_.c_str(), e.what());
    	res.success = false;
    	res.error = e.what();
    }
    return true;
  }

  bool UwdsServerNodelet::getTimeline(uwds_msgs::GetTimeline::Request &req,
                uwds_msgs::GetTimeline::Response &res)
  {
  	if(verbose_)NODELET_INFO("[%s] Client <%s> request 'uwds/get_timeline' in <%s> world", nodelet_name_.c_str(), req.ctxt.client.name.c_str(), req.ctxt.world.c_str());
    try
    {
      if (req.ctxt.client.id != "")
        topology().update(req.ctxt, READ);
    	addChangesPublisher(req.ctxt.world);
    	auto& timeline = worlds()[req.ctxt.world].timeline();
    	std::vector<uwds_msgs::Situation> situations;
      if(timeline.size() > 0)
      {

        for (const auto& situation : timeline.situations())
      	{
      		situations.push_back(*situation);
      	}
      }
    	res.situations = situations;
    	res.origin = timeline.origin();
    	res.success = true;
    	res.error = "";
    }
    catch(const std::exception& e)
    {
    	NODELET_ERROR("[%s] Exception occured while sending timeline for world <%s> to client <%s> : %s", nodelet_name_.c_str(), req.ctxt.world.c_str(), req.ctxt.client.name.c_str(), e.what());
    	res.success = false;
    	res.error = e.what();
    }
    return true;
  }

  bool UwdsServerNodelet::getMesh(uwds_msgs::GetMesh::Request &req,
                                  uwds_msgs::GetMesh::Response &res)
  {
    if(verbose_)NODELET_INFO("[%s] Request 'uwds/get_mesh' <%s>", nodelet_name_.c_str(), req.mesh_id.c_str());
    try
    {
      if (meshes().has(req.mesh_id))
      {
        res.mesh = meshes()[req.mesh_id];
        res.success = true;
      } else {
        NODELET_WARN("Mesh %s not existing",req.mesh_id.c_str());
        res.success = false;
        res.error = (boost::format("Requested mesh <%s> not existing")%req.mesh_id.c_str()).str();
      }
    }
    catch(const std::exception& e)
    {
      NODELET_ERROR("[%s] Exception occured while sending mesh <%s> : %s", nodelet_name_.c_str(), req.mesh_id.c_str(), e.what());
      res.success = false;
      res.error = e.what();
    }
    return true;
  }

  void UwdsServerNodelet::changesCallback(const uwds_msgs::ChangesInContextStampedPtr& msg)
  {
  	if(verbose_)NODELET_INFO("[%s] Received changes from client <%s> in <%s> world", nodelet_name_.c_str(), msg->ctxt.client.name.c_str(), msg->ctxt.world.c_str());
    try
    {
      if (msg->ctxt.client.id != "")
        topology().update(msg->ctxt, WRITE);
    	addChangesPublisher(msg->ctxt.world);

    	auto& scene = worlds()[msg->ctxt.world].scene();
    	auto& timeline = worlds()[msg->ctxt.world].timeline();
    	// remove nodes
      scene.nodes().remove(msg->changes.nodes_to_delete);
    	// remove situations
      timeline.situations().remove(msg->changes.situations_to_delete);
      // remove meshes
      meshes().remove(msg->changes.meshes_to_delete);
      // update meshes
      meshes().update(msg->changes.meshes_to_update);
    	// update nodes
    	for (auto& node : msg->changes.nodes_to_update)
    	{
        if(node.name != "root") // never update a rootnode from clients !
        {
          if(!scene.nodes().has(node.parent))
          {
            // reparent to rootnode if the parent don't exist
            node.parent = scene.rootID();
          }
          // children are lazzely updated (rootnode included)
          std::vector<std::string> parent_children = scene.nodes()[node.parent].children;
          if (std::find(parent_children.begin(), parent_children.end(), node.id) == parent_children.end())
            scene.nodes()[node.parent].children.push_back(node.id);

          // normalize the quaternion ;)
          tf::Quaternion q(node.position.pose.orientation.x,
                           node.position.pose.orientation.y,
                           node.position.pose.orientation.z,
                           node.position.pose.orientation.w);
          q.normalize();
          tf::Pose pose;
          pose.setOrigin(tf::Vector3(node.position.pose.position.x,
                                     node.position.pose.position.y,
                                     node.position.pose.position.z));
          pose.setRotation(q);
          geometry_msgs::Pose pose_msg;
          tf::poseTFToMsg(pose, pose_msg);
          // update the pose according
          node.position.pose = pose_msg;
          // update the node
      		scene.nodes().update(node);
        }
    	}
    	// update situations
      timeline.situations().update(msg->changes.situations_to_update);
    	// broadcast data by publishing the changes into the dedicated world changes topics
      distributeChanges(msg);
    }
    catch(const std::exception& e)
    {
    	NODELET_ERROR("[%s] Exception occured while distributing changes to clients : %s", nodelet_name_.c_str(), e.what());
    }
  }

  void UwdsServerNodelet::cleanUpTimerCallback(const ros::TimerEvent& event)
  {
    float situations_buffer_size;
    pnh_->param<float>("situations_buffer_size",
                       situations_buffer_size,
                       DEFAULT_SITUATIONS_BUFFER_SIZE);
    for(const auto world : worlds())
    {
      Changes changes;
      for(const auto& situation : world->timeline().situations())
      {
        if(situation->end.data != ros::Time())
        {
          if (ros::Time::now() - situation->end.data > ros::Duration(situations_buffer_size))
          {
            changes.situations_to_delete.push_back(situation->id);
          }
        }
      }
      if(changes.situations_to_delete.size()>0)
      {
      world->timeline().situations().remove(changes.situations_to_delete);
      distributeChanges(world->name(), changes);
      }
    }
  }

  void UwdsServerNodelet::distributeChanges(const std::string& world,
                                            const Changes& changes)
  {
    ChangesInContextStampedPtr changes_in_context = boost::make_shared<ChangesInContextStamped>();
    changes_in_context->ctxt.world = world;
    changes_in_context->header.stamp = ros::Time::now();
    changes_in_context->changes = changes;
    distributeChanges(changes_in_context);
  }

  void UwdsServerNodelet::distributeChanges(const ChangesInContextStampedPtr& changes)
  {
    if(verbose_)NODELET_INFO("[%s] Distribute changes to clients connected to world <%s>", nodelet_name_.c_str(), changes->ctxt.world.c_str());
    changes_publishers_map_.at(changes->ctxt.world)->publish(changes);
  }

  void UwdsServerNodelet::addChangesPublisher(const std::string& world)
  {
    addOutputWorld(world);
  	if (changes_publishers_map_.count(world) == 0 )
  	{
  		changes_publishers_map_.emplace(world, boost::make_shared<ros::Publisher>(nh_->advertise<uwds_msgs::ChangesInContextStamped>(world+"/changes", DEFAULT_PUBLISHER_BUFFER_SIZE)));
  		if(verbose_)NODELET_INFO("[%s] Changes publisher for world <%s> created", nodelet_name_.c_str(), world.c_str());
  	}
  }

  void UwdsServerNodelet::removeChangesPublisher(const std::string& world)
  {
    if (changes_publishers_map_.count(world) > 0)
    {
      changes_publishers_map_.erase(world);
      if(verbose_)NODELET_INFO("[%s] Remove changes publisher for world <%s>", nodelet_name_.c_str(), world.c_str());
    }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds::UwdsServerNodelet, nodelet::Nodelet)
