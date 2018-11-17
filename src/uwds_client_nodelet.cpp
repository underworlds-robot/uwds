#include "uwds/uwds_client_nodelet.h"

using namespace std;

namespace uwds
{
	void UwdsClientNodelet::onInit()
	{
		DynamicConnectionBasedNodelet::onInit();
		// General service
		client_id_ = NEW_UUID;
		get_topology_service_client_ = nh_->serviceClient<uwds_msgs::GetTopology>("uwds/get_topology", true);
		NODELET_DEBUG("[%s] Service client 'uwds/get_topology' created", nodelet_name_.c_str());
		// Nodes related service
		get_scene_service_client_ = nh_->serviceClient<uwds_msgs::GetScene>("uwds/get_scene", true);
		NODELET_DEBUG("[%s] Service client 'uwds/get_scene' created", nodelet_name_.c_str());
		// Situations related service
		get_timeline_service_client_ = nh_->serviceClient<uwds_msgs::GetTimeline>("uwds/get_timeline", true);
		NODELET_DEBUG("[%s] Service client 'uwds/get_timeline' created", nodelet_name_.c_str());
		// Meshes related service
		get_mesh_service_client_ = nh_->serviceClient<uwds_msgs::GetMesh>("uwds/get_mesh", true);
		NODELET_DEBUG("[%s] Service client 'uwds/get_mesh' created", nodelet_name_.c_str());
		// Changes publisher
		changes_publisher_ = boost::make_shared<ros::Publisher>(nh_->advertise<uwds_msgs::ChangesInContextStamped>("uwds/changes", publisher_buffer_size_));
		NODELET_DEBUG("[%s] Publisher 'uwds/changes' created", nodelet_name_.c_str());

		pnh_->param<bool>("use_scene", use_scene_, true);
		pnh_->param<bool>("use_timeline", use_timeline_, true);
		pnh_->param<bool>("use_meshes", use_meshes_, true);
	}

	void UwdsClientNodelet::sendWorldChanges(const std::string world,
																					 const std_msgs::Header header,
																					 const uwds_msgs::Changes changes)
	{
		// Create and fill the context header
		uwds_msgs::Context ctxt;
		ctxt.client.name = nodelet_name_;
		ctxt.client.id = client_id_;
		ctxt.client.type = client_type_;
		ctxt.world = world;
		// Prepare the message to be send
		ChangesInContextStampedPtr msg = boost::make_shared<uwds_msgs::ChangesInContextStamped>();
		msg->header = header;
		msg->ctxt = ctxt;
		msg->changes = changes;
	  // Update if not active the output connection
		addOutputWorld(world);
		// Publish the changes to the server using zero-copy pointer passing
		changes_publisher_->publish(msg);
		// Set the flag
		if (ever_send_changes_ == false)
		{
			ever_send_changes_ = true;
		}
	}

	void UwdsClientNodelet::getTopologyFromRemote()
	{
		uwds_msgs::GetTopology get_topology_srv;
		if(!get_topology_service_client_.exists())
		{
			NODELET_ERROR("[%s] Connection to 'uwds/get_topology' service not existing", nodelet_name_.c_str());
			NODELET_ERROR("[%s] Waiting for 'uwds/get_topology' service...", nodelet_name_.c_str());
			ros::service::waitForService("uwds/get_topology", -1);
			get_topology_service_client_ = nh_->serviceClient<uwds_msgs::GetTopology>("uwds/get_topology", false);
		}
		if (get_topology_service_client_.call(get_topology_srv))
		{
			if (get_topology_srv.response.success)
			{
				topology().reset(get_topology_srv.response.worlds, get_topology_srv.response.clients, get_topology_srv.response.client_interactions);
			} else {
				NODELET_ERROR("[%s] Error occured when processing uwds/get_topology : %s",
										 nodelet_name_.c_str(),
										 get_topology_srv.response.error.c_str());
			}
		}
	}

	void UwdsClientNodelet::getSceneFromRemote(const std::string& world)
	{
		if(!use_scene_)
		{
			NODELET_WARN("[%s] Trying to request service 'uwds/get_scene' while '~scene_prefilter' parameter is activated. Skip the request.", nodelet_name_.c_str());
			return;
		}
		// Create and fill the request
		uwds_msgs::GetScene get_scene_srv;
		get_scene_srv.request.ctxt.client.name = nodelet_name_;
		get_scene_srv.request.ctxt.client.id = client_id_;
		get_scene_srv.request.ctxt.client.type = client_type_;
	  get_scene_srv.request.ctxt.world = world;
		// Test the connection to 'uwds/get_scene'
		if(!get_scene_service_client_.exists())
		{
			NODELET_WARN("[%s] Connection to 'uwds/get_scene' service not existing", nodelet_name_.c_str());
			NODELET_WARN("[%s] Waiting for 'uwds/get_scene' service...", nodelet_name_.c_str());
			get_scene_service_client_.waitForExistence(ros::Duration(2.0));
			// If it doesn't exist we create a new connection
			NODELET_WARN("[%s] Try to reconnect to 'uwds/get_scene' service", nodelet_name_.c_str());
			get_scene_service_client_ = nh_->serviceClient<uwds_msgs::GetScene>("uwds/get_scene", true);
			if(!get_scene_service_client_.exists())
			{
				NODELET_ERROR("[%s] Connection to 'uwds/get_scene' failed, skip the request", nodelet_name_.c_str());
				return;
			}
		}
		// Make the request to the server
		if (get_scene_service_client_.call(get_scene_srv))
	  {
			if (get_scene_srv.response.success)
			{
				// If the service request is a success
				// We reset the scene with the correct rootnode ID (the one set by the server)
				worlds()[world].scene().reset(get_scene_srv.response.root_id);
				// And update the nodes received
				for(auto& node : get_scene_srv.response.nodes)
				{
					if (use_meshes_)
					{ //fetch the meshes of the node
						getNodeMeshes(node);
					}
					worlds()[world].scene().nodes().update(node);
				}
			} else {
				// Prompt error message if service failed
				NODELET_ERROR("[%s] Error occured when processing uwds/get_scene for world <%s> : %s",
										 nodelet_name_.c_str(),
										 world.c_str(),
										 get_scene_srv.response.error.c_str());
			}
	  }
	}

	void UwdsClientNodelet::getTimelineFromRemote(const std::string& world)
	{
		if(!use_timeline_) // Check the prefilter
		{
			NODELET_WARN("[%s] Trying to request service 'uwds/get_timeline' while '~timeline_prefilter' parameter is activated. Skip the request.", nodelet_name_.c_str());
			return;
		}
		// Create and fill the request
		uwds_msgs::GetTimeline get_timeline_srv;
		get_timeline_srv.request.ctxt.client.name = nodelet_name_;
		get_timeline_srv.request.ctxt.client.id = client_id_;
		get_timeline_srv.request.ctxt.client.type = client_type_;
	  get_timeline_srv.request.ctxt.world = world;

		// We check that the service 'uwds/get_timeline' service exist
		if(!get_timeline_service_client_.exists())
		{
			NODELET_ERROR("[%s] Connection to 'uwds/get_timeline' service not existing", nodelet_name_.c_str());
			NODELET_ERROR("[%s] Waiting for 'uwds/get_timeline' service...", nodelet_name_.c_str());
			get_timeline_service_client_.waitForExistence(ros::Duration(2.0));
			// If it doesn't exist we create a new connection
			NODELET_WARN("[%s] Try to reconnect to 'uwds/get_timeline' service", nodelet_name_.c_str());
			get_timeline_service_client_ = nh_->serviceClient<uwds_msgs::GetTimeline>("uwds/get_timeline", false);
			if(!get_timeline_service_client_.exists())
			{
				NODELET_ERROR("[%s] Connection to 'uwds/get_timeline' failed, skip the request", nodelet_name_.c_str());
				return;
			}
		}
		// Make the request to the server
	  if (get_timeline_service_client_.call(get_timeline_srv))
	  {
			if (get_timeline_srv.response.success)
			{
				// If the service request is a success
				// We reset the timeline with the correct origin (the one set by the server)
				worlds()[world].timeline().reset(get_timeline_srv.response.origin.data);
				worlds()[world].timeline().situations().update(get_timeline_srv.response.situations);
			} else {
				// Prompt error message if service failed
				NODELET_ERROR("[%s] Error occured when processing uwds/get_timeline for world <%s> : %s",
										 nodelet_name_.c_str(),
										 world.c_str(),
										 get_timeline_srv.response.error.c_str());
			}
	  }
	}

	void UwdsClientNodelet::initializeWorld(const std::string& world)
	{
		getSceneFromRemote(world);
		getTimelineFromRemote(world);
	}

	bool UwdsClientNodelet::getMeshFromRemote(const std::string& mesh_id)
	{
		if(!use_meshes_)
		{
			NODELET_WARN("[%s] Trying to request service 'uwds/get_mesh' while '~use_mesh' parameter is false.", nodelet_name_.c_str());
		}
		uwds_msgs::GetMesh get_mesh_srv;
		get_mesh_srv.request.mesh_id = mesh_id;

		if (meshes().has(mesh_id)){
			return true;
		} else {

			if(!get_mesh_service_client_.exists())
			{
				NODELET_WARN("[%s] Connection to 'uwds/get_mesh' service not existing", nodelet_name_.c_str());
				NODELET_WARN("[%s] Waiting for 'uwds/get_mesh' service...", nodelet_name_.c_str());
				ros::service::waitForService("uwds/get_mesh", -1);
				get_mesh_service_client_ = nh_->serviceClient<uwds_msgs::GetMesh>("uwds/get_mesh", false);
			}
			if(get_mesh_service_client_.call(get_mesh_srv))
			{
				if (!get_mesh_srv.response.success)
				{
					NODELET_ERROR("[%s] Error occured when processing 'uwds/get_mesh' : %s",
											nodelet_name_.c_str(),
											get_mesh_srv.response.error.c_str());
					return false;
				}
			} else {
				NODELET_ERROR("[%s] Error occured when calling 'uwds/get_mesh'",
										nodelet_name_.c_str());
				return false;
			}
			NODELET_DEBUG("[%s] Update mesh <%s> in local data-structure",
									nodelet_name_.c_str(),
									mesh_id.c_str());
			meshes().update(get_mesh_srv.response.mesh);
			return true;
		}
	}

	std::vector<std::string> UwdsClientNodelet::getNodeMeshes(const uwds_msgs::Node& node)
	{
		std::vector<std::string> mesh_ids_list;
		std::vector<std::string> mesh_ids;
		if (node.name!= "root" && node.type == MESH)
		{
			for(const auto& property : node.properties)
			{
				if(property.name == "meshes" && !property.data.empty())
				{
					std::vector<std::string> mesh_ids;
					boost::split(mesh_ids, property.data, boost::is_any_of(","), boost::token_compress_on);
					for (const auto mesh_id : mesh_ids)
					{
						if(getMeshFromRemote(mesh_id))
							mesh_ids_list.push_back(mesh_id);
					}
					return mesh_ids_list;
				}
			}
		}
		return mesh_ids_list;
	}
}
