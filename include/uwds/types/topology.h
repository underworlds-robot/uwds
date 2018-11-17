#ifndef TOPOLOGY_HPP
#define TOPOLOGY_HPP

#include<string>
#include<vector>
#include<map>
#include<mutex>

#include<ros/ros.h>

#include "concurrent_container.h"
#include <uwds_msgs/Client.h>
#include <uwds_msgs/ClientInteraction.h>

#include <std_msgs/Time.h>

using namespace uwds_msgs;

namespace uwds {

  /** @brief
   * Enum to represent the client interaction types
   */
  enum ClientInteractionType {
    READ = 0,
    WRITE
  };
  /** @brief
   * The client interaction type names
   */
  static const std::array<std::string,2> ClientInteractionTypeName{"read",
                                                                   "write"};
  /** @brief
   * This class represent the Underworlds clients topology
   */
  class Topology {
    public:
      Topology() {}

      ~Topology() {}

      void update(const Context& ctxt, const ClientInteractionType& type)
      {
        ros::Time current_time = ros::Time::now();
        if(std::find(worlds_.begin(), worlds_.end(), ctxt.world) == worlds_.end())
        {
          this->lock();
          worlds_.push_back(ctxt.world);
          this->unlock();
        }
        if(!clients_.has(ctxt.client.id))
          clients_.update(ctxt.client.id, ctxt.client);

        if(!client_interactions_.has(ctxt.client.id))
        {
          uwds_msgs::ClientInteraction interaction_msg;
          interaction_msg.client_id = ctxt.client.id;
          interaction_msg.world = ctxt.world;
          interaction_msg.type = type;
          interaction_msg.last_activity.data = current_time;
          std::vector<uwds_msgs::ClientInteraction> interactions;
          interactions.push_back(interaction_msg);
          client_interactions_.update(ctxt.client.id, interactions);
        } else {
          std::vector<ClientInteraction> interactions = client_interactions_[ctxt.client.id];
          for (auto interaction : client_interactions_[ctxt.client.id])
          {
            if(interaction.client_id == ctxt.client.id
                && interaction.world == ctxt.world
                && interaction.type == type)
            {
              interaction.last_activity.data = current_time;
            } else {
              uwds_msgs::ClientInteraction interaction_msg;
              interaction_msg.client_id = ctxt.client.id;
              interaction_msg.world = ctxt.world;
              interaction_msg.type = type;
              interaction_msg.last_activity.data = current_time;
              interactions.push_back(interaction_msg);
            }
          }
          client_interactions_.update(ctxt.client.id, interactions);
        }
      }

      std::vector<std::string> worlds()
      {
        return worlds_;
      }

      ConcurrentContainer<Client>& clients()
      {
        return clients_;
      }

      ConcurrentContainer<std::vector<uwds_msgs::ClientInteraction>>& client_interactions()
      {
        return client_interactions_;
      }

      void reset() {
        this->lock();
        worlds_.clear();
        clients_.reset();
        client_interactions_.reset();
        this->unlock();
      }

      void reset(const std::vector<std::string> worlds,
                  const std::vector<uwds_msgs::Client> clients,
                  const std::vector<uwds_msgs::ClientInteraction> client_interactions)
      {
        this->lock();
        worlds_.clear();
        clients_.reset();
        client_interactions_.reset();
        for(const auto& world : worlds)
        {
          worlds_.push_back(world);
        }
        for(const auto& client : clients)
        {
          clients_.update(client.id, client);
        }
        for(const auto& client_interaction : client_interactions)
        {
          if (client_interactions_.has(client_interaction.client_id))
          {
            client_interactions_[client_interaction.client_id].push_back(client_interaction);
          } else {
            std::vector<uwds_msgs::ClientInteraction> interactions;
            interactions.push_back(client_interaction);
            client_interactions_.update(client_interaction.client_id, interactions);
          }
        }
        this->unlock();
      }

      void lock() {mutex_.lock();}
      void unlock() {mutex_.unlock();}

    private:

      std::mutex mutex_;

      std::vector<std::string> worlds_;

      ConcurrentContainer<Client> clients_;

      ConcurrentContainer<std::vector<uwds_msgs::ClientInteraction>> client_interactions_;
  };

  typedef boost::shared_ptr<uwds::Topology> TopologyPtr;
  typedef boost::shared_ptr<uwds::Topology const> TopologyConstPtr;
}

#endif
