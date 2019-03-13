#ifndef TOPOLOGY_HPP
#define TOPOLOGY_HPP

#include<string>
#include<vector>
#include<map>
#include<mutex>

#include<ros/ros.h>

#include "concurrent_container.h"
#include <uwds_msgs/Connection.h>
#include <uwds_msgs/Client.h>
#include <uwds_msgs/ClientInteraction.h>
#include <std_msgs/Time.h>

using namespace uwds_msgs;

namespace uwds {

  typedef ConcurrentContainer<Client> Clients;
  typedef boost::shared_ptr<Clients> ClientsPtr;

  typedef ConcurrentContainer<ClientInteraction> ClientInteractions;
  typedef boost::shared_ptr<ClientInteractions> ClientInteractionsPtr;

  typedef ConcurrentContainer<ClientInteractions> ClientInteractionsByWorld;
  typedef boost::shared_ptr<ClientInteractionsByWorld> ClientInteractionsByWorldPtr;

  /** @brief
   * The interaction types.
   */
  enum ConnectionInteractionType {
    READ = uwds_msgs::Connection::READ,
    WRITE = uwds_msgs::Connection::WRITE
  };

  /** @brief
   * The connection actions types.
   */
  enum ConnectionActionType {
    CONNECT = uwds_msgs::Connection::CONNECT,
    DISCONNECT = uwds_msgs::Connection::DISCONNECT
  };

  /** @brief
   * This class represent the Underworlds clients topology
   */
  class Topology {
    public:
      Topology()
      {
        clients_ = boost::make_shared<Clients>();
        client_interactions_by_world_ = boost::make_shared<ClientInteractionsByWorld>();
      }

      ~Topology() {}

      Clients& clients() {return *clients_;}

      ClientInteractionsByWorld& clientsInteractions() {return *client_interactions_by_world_;}

      ClientInteractions& clientInteractionsByWorld(const std::string& world_name)
      {
        return (*client_interactions_by_world_)[world_name];
      }

      void update(const Context& ctxt,
                  const ConnectionInteractionType& interaction_type,
                  const ConnectionActionType& action_type)
      {
        this->lock();
        ros::Time current_time = ros::Time::now();

        if (action_type == CONNECT)
        {
          clients_->update(ctxt.client.id, ctxt.client);

          if(!client_interactions_by_world_->has(ctxt.world))
          {
            ClientInteractionsPtr interactions = boost::make_shared<ClientInteractions>();
            client_interactions_by_world_->update(ctxt.world, interactions);
            ClientInteraction interaction_msg;
            interaction_msg.ctxt = ctxt;
            interaction_msg.type = interaction_type;
            clientInteractionsByWorld(interaction_msg.ctxt.world).update(interaction_msg.ctxt.client.id, interaction_msg);
          } else {
            uwds_msgs::ClientInteraction interaction_msg;
            interaction_msg.ctxt = ctxt;
            interaction_msg.type = interaction_type;
            clientInteractionsByWorld(ctxt.world).update(interaction_msg.ctxt.client.id, interaction_msg);
          }
        } else {
          clientInteractionsByWorld(ctxt.world).remove(ctxt.client.id);
        }
        this->unlock();
      }

      void reset() {
        this->lock();
        clients_->reset();
        client_interactions_by_world_->reset();
        this->unlock();
      }

      void reset(const std::vector<std::string> worlds,
                  const std::vector<uwds_msgs::Client> clients,
                  const std::vector<uwds_msgs::ClientInteraction> client_interactions)
      {
        reset();
        this->lock();
        for(const auto& client : clients)
        {
          clients_->update(client.id, client);
        }
        for(const auto& client_interaction : client_interactions)
        {
          if (!client_interactions_by_world_->has(client_interaction.ctxt.world))
          {
            ClientInteractionsPtr interactions = boost::make_shared<ClientInteractions>();
            client_interactions_by_world_->update(client_interaction.ctxt.world, interactions);
            clientInteractionsByWorld(client_interaction.ctxt.world).update(client_interaction.ctxt.client.id, client_interaction);
          } else {
            clientInteractionsByWorld(client_interaction.ctxt.world).update(client_interaction.ctxt.client.id, client_interaction);
          }
        }
        this->unlock();
      }

      void lock() {mutex_.lock();}
      void unlock() {mutex_.unlock();}

    private:

      std::mutex mutex_;

      ClientsPtr clients_;

      ClientInteractionsByWorldPtr client_interactions_by_world_;
  };

  typedef boost::shared_ptr<uwds::Topology> TopologyPtr;
  typedef boost::shared_ptr<uwds::Topology const> TopologyConstPtr;
}

#endif
