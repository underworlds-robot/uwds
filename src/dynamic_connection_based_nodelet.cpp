#include "uwds/dynamic_connection_based_nodelet.h"

using namespace std;

namespace uwds
{

  void DynamicConnectionBasedNodelet::onInit()
  {
    //ros::param::param<std::string>("global_frame_id", global_frame_id_, "map");
    ros::param::param<bool>("~verbose", verbose_, true);
    connection_status_ = NOT_INITIALIZED;
    bool use_multithread;
    ros::param::param<bool>("~use_multithread_callback", use_multithread, true);
    nodelet_name_ = getName().substr(1, std::string::npos);

    if (use_multithread) {
      if(verbose_)NODELET_INFO("[%s] Use multithread callback", nodelet_name_.c_str());
      nh_ = boost::make_shared<ros::NodeHandle>(getMTNodeHandle());
      pnh_ = boost::make_shared<ros::NodeHandle>(getMTPrivateNodeHandle());
    } else {
      if(verbose_)NODELET_INFO("[%s] Use singlethread callback", nodelet_name_.c_str());
      nh_ = boost::make_shared<ros::NodeHandle>(getNodeHandle());
      pnh_ = boost::make_shared<ros::NodeHandle>(getPrivateNodeHandle());
    }
    pnh_->param<int>("publisher_buffer_size", publisher_buffer_size_ , DEFAULT_PUBLISHER_BUFFER_SIZE);
    if (publisher_buffer_size_ == DEFAULT_PUBLISHER_BUFFER_SIZE)
      if(verbose_)NODELET_INFO("[%s] Using default '~publisher_buffer_size' : %d", nodelet_name_.c_str(), DEFAULT_PUBLISHER_BUFFER_SIZE);
  	pnh_->param<int>("subscriber_buffer_size", subscriber_buffer_size_ , DEFAULT_SUBSCRIBER_BUFFER_SIZE);
    if (subscriber_buffer_size_ == DEFAULT_SUBSCRIBER_BUFFER_SIZE)
      if(verbose_)NODELET_INFO("[%s] Using default parameter '~subscriber_buffer_size' : %d", nodelet_name_.c_str(), DEFAULT_SUBSCRIBER_BUFFER_SIZE);
    pnh_->param<int>("time_synchronizer_buffer_size", subscriber_buffer_size_ , DEFAULT_TIME_SYNCHRONIZER_BUFFER_SIZE);
    if (time_synchronizer_buffer_size_ == DEFAULT_TIME_SYNCHRONIZER_BUFFER_SIZE)
      if(verbose_)NODELET_INFO("[%s] Using default '~time_synchronizer_buffer_size' : %d", nodelet_name_.c_str(), DEFAULT_TIME_SYNCHRONIZER_BUFFER_SIZE);

    list_input_worlds_server_ = nh_->advertiseService(nodelet_name_+"/input_worlds", &DynamicConnectionBasedNodelet::listInputWorlds, this);
    if(verbose_)NODELET_INFO("[%s] Service '~input_worlds' advertised", nodelet_name_.c_str());
    list_output_worlds_server_ = nh_->advertiseService(nodelet_name_+"/output_worlds", &DynamicConnectionBasedNodelet::listOutputWorlds, this);
    if(verbose_)NODELET_INFO("[%s] Service '~output_worlds' advertised", nodelet_name_.c_str());
  }

  void DynamicConnectionBasedNodelet::resetInputWorlds()
  {
  	input_worlds_.clear();
  }

  void DynamicConnectionBasedNodelet::resetOutputWorlds()
  {
  	output_worlds_.clear();
  }

  void DynamicConnectionBasedNodelet::addInputWorld(const std::string& world)
  {
    if (std::find(input_worlds_.begin(), input_worlds_.end(), world) == input_worlds_.end())
      input_worlds_.push_back(world);
  }

  void DynamicConnectionBasedNodelet::addOutputWorld(const std::string& world)
  {
    if (std::find(output_worlds_.begin(), output_worlds_.end(), world) == output_worlds_.end())
      output_worlds_.push_back(world);
  }

  bool DynamicConnectionBasedNodelet::listInputWorlds(
                                           uwds_msgs::List::Request &req,
  																				 uwds_msgs::List::Response &res)
  {
    if(verbose_)NODELET_INFO("[%s] Service '~input_worlds' requested", nodelet_name_.c_str());
    res.success = false;
    switch (connection_status_) {
      case NOT_INITIALIZED : res.error = "Nodelet not initialized, try later !"; break;
      case NOT_CONNECTED : res.error = "Nodelet not connected, try to reconfigure it !"; break;
      case CONNECTING : res.error = "Nodelet currently connecting, try later !"; break;
      default:
        res.success = true;
        res.list = input_worlds_;
    }
  	return true;
  }

  bool DynamicConnectionBasedNodelet::listOutputWorlds(
                                           uwds_msgs::List::Request &req,
  																				 uwds_msgs::List::Response &res)
  {
    if(verbose_)NODELET_INFO("[%s] Service '~output_worlds' requested", nodelet_name_.c_str());
    res.success = false;
    switch (connection_status_) {
      case NOT_INITIALIZED : res.error = "Nodelet not initialized, try later !"; break;
      case NOT_CONNECTED : res.error = "Nodelet not connected, try to reconfigure it !"; break;
      case CONNECTING : res.error = "Nodelet currently connecting, try later !"; break;
      default:
        res.success = true;
        res.list = output_worlds_;
    }
  	return true;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds::DynamicConnectionBasedNodelet, nodelet::Nodelet)
