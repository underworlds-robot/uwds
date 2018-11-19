#include "uwds/reconfigurable_client.h"

using namespace uwds;
using namespace message_filters;

namespace uwds
{
  void ReconfigurableClient::onInit()
  {
    UwdsClientNodelet::onInit();
    // Parameters
    pnh_->param<bool>("synchronized", synchronized_, false);

    uwds_msgs::ReconfigureInputs reconfigure_srv;

    std::string default_inputs;
    pnh_->param<std::string>("default_inputs", default_inputs, "");
    if (default_inputs!="")
    {
      std::vector<std::string> default_inputs_list;
      boost::split(default_inputs_list, default_inputs, boost::is_any_of(" "), boost::token_compress_on);
      try {
        reconfigure(default_inputs_list);
      } catch (const std::invalid_argument& e) {
        NODELET_ERROR("[%s] Error occured : %s", nodelet_name_.c_str(), e.what());
        connection_status_ = NOT_CONNECTED;
      }
    } else {
      connection_status_ = NOT_CONNECTED;
    }
    //Services
    reconfigure_service_server_ = nh_->advertiseService(nodelet_name_+"/reconfigure_inputs", &ReconfigurableClient::reconfigureInputs, this);
    if(verbose_)NODELET_INFO("[%s] Service '~reconfigure_inputs' advertised", nodelet_name_.c_str());
  }

  bool ReconfigurableClient::reconfigure(const std::vector<std::string>& new_input_worlds)
  {
    connection_mutex_.lock();
    if (new_input_worlds.size()==0)
    {
      //disconnect the nodelet
      if (connection_status_ == CONNECTED)
      {
        NODELET_WARN("[%s] Disconnecting the nodelet", nodelet_name_.c_str());
        if (synchronized_)
          active_sync_connection_.disconnect();
        for (auto& input_world : input_worlds_)
        {
          removeChangesSubscriber(input_world);
          onUnsubscribeChanges(input_world);
        }
        resetInputWorlds();
        resetOutputWorlds();
        connection_status_ = NOT_CONNECTED;
      }
      connection_mutex_.unlock();
      return true;
    }
    connection_status_ = CONNECTING;
    if (synchronized_)
    {
      if (new_input_worlds.size() > 8 || new_input_worlds.size() < 2)
      {
        connection_mutex_.unlock();
        throw std::invalid_argument("Incorrect number of inputs specified");
        return false;
      }
    }
    // We unsubscribe from current input worlds
    for (auto& input_world : input_worlds_)
    {
      removeChangesSubscriber(input_world);
      onUnsubscribeChanges(input_world);
    }
    // Then we reset the input worlds & output world track lists
    resetInputWorlds();
    resetOutputWorlds();
    // To finally subscribe to new inputs
    for (const auto& new_input_world : new_input_worlds)
    {
      initializeWorld(new_input_world);
      addChangesSubscriber(new_input_world);
      onSubscribeChanges(new_input_world);
    }

    if (synchronized_)
    {
      switch (new_input_worlds.size()) {
        case 2:
          if (time_synchronizer2 != NULL)
            delete time_synchronizer2;
          time_synchronizer2 = new TimeSynchronizer<uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped>(
                                       *sync_changes_subscribers_map_[new_input_worlds[0]],
                                       *sync_changes_subscribers_map_[new_input_worlds[1]],
                                       time_synchronizer_buffer_size_);
          active_sync_connection_ = time_synchronizer2->registerCallback(
            boost::bind(&ReconfigurableClient::changesCallback, this, _1, _2));
          break;

        case 3:
          if (time_synchronizer3 != NULL)
            delete time_synchronizer3;
          time_synchronizer3 = new TimeSynchronizer<uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped>(
                                        *sync_changes_subscribers_map_[new_input_worlds[0]],
                                        *sync_changes_subscribers_map_[new_input_worlds[1]],
                                        *sync_changes_subscribers_map_[new_input_worlds[2]],
                                        time_synchronizer_buffer_size_);
          active_sync_connection_ = time_synchronizer3->registerCallback(
            boost::bind(&ReconfigurableClient::changesCallback, this, _1, _2, _3));
          break;

        case 4:
          if (time_synchronizer4 != NULL)
            delete time_synchronizer4;
          time_synchronizer4 = new TimeSynchronizer<uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped>(
                                          *sync_changes_subscribers_map_[new_input_worlds[0]],
                                          *sync_changes_subscribers_map_[new_input_worlds[1]],
                                          *sync_changes_subscribers_map_[new_input_worlds[2]],
                                          *sync_changes_subscribers_map_[new_input_worlds[3]],
                                          time_synchronizer_buffer_size_);
          active_sync_connection_ = time_synchronizer4->registerCallback(
            boost::bind(&ReconfigurableClient::changesCallback, this, _1, _2, _3, _4));
          break;

        case 5:
          if (time_synchronizer5 != NULL)
            delete time_synchronizer5;
          time_synchronizer5 = new TimeSynchronizer<uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped>(
                                          *sync_changes_subscribers_map_[new_input_worlds[0]],
                                          *sync_changes_subscribers_map_[new_input_worlds[1]],
                                          *sync_changes_subscribers_map_[new_input_worlds[2]],
                                          *sync_changes_subscribers_map_[new_input_worlds[3]],
                                          *sync_changes_subscribers_map_[new_input_worlds[4]],
                                          time_synchronizer_buffer_size_);
          active_sync_connection_ = time_synchronizer5->registerCallback(
            boost::bind(&ReconfigurableClient::changesCallback, this, _1, _2, _3, _4, _5));
          break;

        case 6:
          if (time_synchronizer6 != NULL)
            delete time_synchronizer6;
          time_synchronizer6 = new TimeSynchronizer<uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped>(
                                        *sync_changes_subscribers_map_[new_input_worlds[0]],
                                        *sync_changes_subscribers_map_[new_input_worlds[1]],
                                        *sync_changes_subscribers_map_[new_input_worlds[2]],
                                        *sync_changes_subscribers_map_[new_input_worlds[3]],
                                        *sync_changes_subscribers_map_[new_input_worlds[4]],
                                        *sync_changes_subscribers_map_[new_input_worlds[5]],
                                        time_synchronizer_buffer_size_);
          active_sync_connection_ = time_synchronizer6->registerCallback(
            boost::bind(&ReconfigurableClient::changesCallback, this, _1, _2, _3, _4, _5, _6));
          break;

        case 7:
          if (time_synchronizer7 != NULL)
            delete time_synchronizer7;
          time_synchronizer7 = new TimeSynchronizer<uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped>(
                                          *sync_changes_subscribers_map_[new_input_worlds[0]],
                                          *sync_changes_subscribers_map_[new_input_worlds[1]],
                                          *sync_changes_subscribers_map_[new_input_worlds[2]],
                                          *sync_changes_subscribers_map_[new_input_worlds[3]],
                                          *sync_changes_subscribers_map_[new_input_worlds[4]],
                                          *sync_changes_subscribers_map_[new_input_worlds[5]],
                                          *sync_changes_subscribers_map_[new_input_worlds[6]],
                                          time_synchronizer_buffer_size_);
          active_sync_connection_ = time_synchronizer7->registerCallback(
            boost::bind(&ReconfigurableClient::changesCallback, this, _1, _2, _3, _4, _5, _6, _7));
          break;

        case 8:
          if (time_synchronizer8 != NULL)
            delete time_synchronizer8;
          time_synchronizer8 = new TimeSynchronizer<uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped, uwds_msgs::ChangesInContextStamped>(
                                          *sync_changes_subscribers_map_[new_input_worlds[0]],
                                          *sync_changes_subscribers_map_[new_input_worlds[1]],
                                          *sync_changes_subscribers_map_[new_input_worlds[2]],
                                          *sync_changes_subscribers_map_[new_input_worlds[3]],
                                          *sync_changes_subscribers_map_[new_input_worlds[4]],
                                          *sync_changes_subscribers_map_[new_input_worlds[5]],
                                          *sync_changes_subscribers_map_[new_input_worlds[6]],
                                          *sync_changes_subscribers_map_[new_input_worlds[7]],
                                          time_synchronizer_buffer_size_);
          active_sync_connection_ = time_synchronizer8->registerCallback(
            boost::bind(&ReconfigurableClient::changesCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));
          break;
      }
    }
    onReconfigure(new_input_worlds);
    connection_status_ = CONNECTED;
    if (ever_connected_ == false)
      ever_connected_ = true;
    connection_mutex_.unlock();
    return true;
  }

  bool ReconfigurableClient::reconfigureInputs(uwds_msgs::ReconfigureInputs::Request &req,
                                               uwds_msgs::ReconfigureInputs::Response &res)
  {
    if(verbose_)NODELET_INFO("[%s] Service '~reconfigure_inputs' requested", nodelet_name_.c_str());
    try
    {
      res.success = reconfigure(req.inputs);
    } catch( const std::invalid_argument& e ) {
      res.error = e.what();
      res.success = false;
    }
  }

  void ReconfigurableClient::applyChanges(
    const std::vector<uwds_msgs::ChangesInContextStampedConstPtr>& changes_list)
  {
    for (const auto& changes_in_ctxt : changes_list)
    {
      Invalidations invalidations = worlds()[changes_in_ctxt->ctxt.world].applyChanges(changes_in_ctxt->header, changes_in_ctxt->changes);
      onChanges(changes_in_ctxt->ctxt.world, changes_in_ctxt->header, invalidations);
    }
  }

  void ReconfigurableClient::changesCallback(
    const uwds_msgs::ChangesInContextStampedConstPtr& msg)
  {
    std::vector<uwds_msgs::ChangesInContextStampedConstPtr> msgs;
    msgs.push_back(msg);
    applyChanges(msgs);
  }

  void ReconfigurableClient::changesCallback(
    const uwds_msgs::ChangesInContextStampedConstPtr& msg0,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg1)
  {
    std::vector<uwds_msgs::ChangesInContextStampedConstPtr> msgs;
    msgs.push_back(msg0);
    msgs.push_back(msg1);
    applyChanges(msgs);
  }

  void ReconfigurableClient::changesCallback(
    const uwds_msgs::ChangesInContextStampedConstPtr& msg0,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg1,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg2)
  {
    std::vector<uwds_msgs::ChangesInContextStampedConstPtr> msgs;
    msgs.push_back(msg0);
    msgs.push_back(msg1);
    msgs.push_back(msg2);
    applyChanges(msgs);
  }

  void ReconfigurableClient::changesCallback(
    const uwds_msgs::ChangesInContextStampedConstPtr& msg0,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg1,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg2,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg3)
  {
    std::vector<uwds_msgs::ChangesInContextStampedConstPtr> msgs;
    msgs.push_back(msg0);
    msgs.push_back(msg1);
    msgs.push_back(msg2);
    msgs.push_back(msg3);
    applyChanges(msgs);
  }

  void ReconfigurableClient::changesCallback(
    const uwds_msgs::ChangesInContextStampedConstPtr& msg0,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg1,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg2,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg3,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg4)
  {
    std::vector<uwds_msgs::ChangesInContextStampedConstPtr> msgs;
    msgs.push_back(msg0);
    msgs.push_back(msg1);
    msgs.push_back(msg2);
    msgs.push_back(msg3);
    msgs.push_back(msg4);
    applyChanges(msgs);
  }

  void ReconfigurableClient::changesCallback(
    const uwds_msgs::ChangesInContextStampedConstPtr& msg0,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg1,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg2,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg3,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg4,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg5)
  {
    std::vector<uwds_msgs::ChangesInContextStampedConstPtr> msgs;
    msgs.push_back(msg0);
    msgs.push_back(msg1);
    msgs.push_back(msg2);
    msgs.push_back(msg3);
    msgs.push_back(msg4);
    msgs.push_back(msg5);
    applyChanges(msgs);
  }

  void ReconfigurableClient::changesCallback(
    const uwds_msgs::ChangesInContextStampedConstPtr& msg0,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg1,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg2,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg3,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg4,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg5,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg6)
  {
    std::vector<uwds_msgs::ChangesInContextStampedConstPtr> msgs;
    msgs.push_back(msg0);
    msgs.push_back(msg1);
    msgs.push_back(msg2);
    msgs.push_back(msg3);
    msgs.push_back(msg4);
    msgs.push_back(msg5);
    msgs.push_back(msg6);
    applyChanges(msgs);
  }

  void ReconfigurableClient::changesCallback(
    const uwds_msgs::ChangesInContextStampedConstPtr& msg0,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg1,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg2,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg3,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg4,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg5,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg6,
    const uwds_msgs::ChangesInContextStampedConstPtr& msg7)
  {
    std::vector<uwds_msgs::ChangesInContextStampedConstPtr> msgs;
    msgs.push_back(msg0);
    msgs.push_back(msg1);
    msgs.push_back(msg2);
    msgs.push_back(msg3);
    msgs.push_back(msg4);
    msgs.push_back(msg5);
    msgs.push_back(msg6);
    msgs.push_back(msg7);
    applyChanges(msgs);
  }

  void ReconfigurableClient::addChangesSubscriber(const std::string& world)
  {
    addInputWorld(world);
    bool added = false;
    if (synchronized_){
      if (sync_changes_subscribers_map_.count(world) == 0)
        sync_changes_subscribers_map_.emplace(world, boost::make_shared<message_filters::Subscriber<uwds_msgs::ChangesInContextStamped>>(*nh_, world+"/changes", publisher_buffer_size_));
        added = true;
    } else {
      if (changes_subscribers_map_.count(world) == 0)
        changes_subscribers_map_.emplace(world, boost::make_shared<ros::Subscriber>(nh_->subscribe(world+"/changes", publisher_buffer_size_, &ReconfigurableClient::changesCallback, this)));
        added = true;
    }
    if (added)
      if(verbose_)NODELET_INFO("[%s] Changes subscriber for world <%s> created", nodelet_name_.c_str(), world.c_str());
  }

  void ReconfigurableClient::removeChangesSubscriber(const std::string& world)
  {
    bool removed = false;
    if (synchronized_){
      if (sync_changes_subscribers_map_.count(world) > 0)
      {
        sync_changes_subscribers_map_.erase(world);
        removed = true;
      }
    } else {
      if (changes_subscribers_map_.count(world) > 0)
      {
        changes_subscribers_map_.erase(world);
        removed = true;
      }
    }
    if (removed)
      if(verbose_)NODELET_INFO("[%s] Remove changes subscriber for world <%s>", nodelet_name_.c_str(), world.c_str());
  }
}
