#ifndef NODES_HPP
#define NODES_HPP

#include<string>
#include<array>
#include<map>
#include<mutex>

#include "concurrent_container.h"
#include <uwds_msgs/Node.h>

using namespace uwds_msgs;

namespace uwds {

  #define NEW_UUID boost::uuids::to_string(boost::uuids::random_generator()())

  /** @brief
   * The node types enum
   */
  enum NodeType {
    ENTITY = uwds_msgs::Node::ENTITY,
    MESH = uwds_msgs::Node::MESH,
    CAMERA = uwds_msgs::Node::CAMERA
  };
  /** @brief
   * The types names corresponding
   */
  static const std::array<std::string,3> NodeTypeName{"entity", "mesh", "camera"};

  /** @brief
   * This class represent the Underworlds nodes container
   */
  class Nodes : public ConcurrentContainer<Node> {

    using ConcurrentContainer::update;

    public:

      /** @brief
       * This method update a node (or create one if new)
       *
       * @param node The node to update
       */
      void update(const NodePtr node) {
        update(node->id, node);
      }

      /** @brief
       * This method update a node (or create one if new)
       *
       * @param node The node to update
       */
      void update(const Node node) {
        update(node.id, node);
      }

      /** @brief
       * This method update a set of nodes (or create them if new)
       *
       * @param nodes The nodes to update
       */
      void update(const std::vector<Node> nodes)
      {
        for(const auto& node : nodes)
        {
          update(node);
        }
      }

      /** @brief
       * This method update a node (or create one if new)
       *
       * @param nodes The nodes to update
       */
      void update(const std::vector<NodePtr> nodes)
      {
        for(const auto& node : nodes)
        {
          update(node);
        }
      }

      std::string getNodeProperty(const std::string& node_id, const std::string& property_name)
      {
        this->lock();
        for(const auto& property : (*this)[node_id].properties)
        {
          if (property.name == property_name)
          {
            this->unlock();
            return property.data;
          }
        }
        this->unlock();
        return "";
      }

       /** @brief
        * Returns the nodes by name
        *
        * @param property_name The property name to test
        */
       std::vector<NodePtr> byProperty(const std::string& property_name)
       {
         std::vector<NodePtr> nodes;
         std::string property;
         this->lock();
         for(const auto node : *this)
         {
           property = getNodeProperty(node->id, property_name);
           if(property != "")
             nodes.push_back(node);
         }
         this->unlock();
         return nodes;
       }

       /** @brief
        * Returns the nodes by property
        *
        * @param property_name The property name to test
        * @param property_data The property data to test
        */
       std::vector<NodePtr> byProperty(const std::string& property_name, const std::string& property_data)
       {
         std::vector<NodePtr> nodes;
         std::string property;
         this->lock();
         for(const auto node : *this)
         {
           property = getNodeProperty(node->id, property_name);
           if(property == property_data)
             nodes.push_back(node);
         }
         this->unlock();
         return nodes;
       }

       /** @brief
        * Returns the nodes by name
        *
        * @param name The name to test
        */
       std::vector<NodePtr> byName(const std::string& name)
       {
         std::vector<NodePtr> nodes;
         this->lock();
         for(const auto node : *this)
         {
           if(node->name == name)
           {
             nodes.push_back(node);
           }
         }
         this->unlock();
         return nodes;
       }

       /** @brief
        * Returns the nodes by type
        *
        * @param type The type to test
        */
       std::vector<NodePtr> byType(const NodeType& type)
       {
         std::vector<NodePtr> nodes;
         this->lock();
         for(const auto& node : *this)
         {
           if(node->type == type)
           {
             nodes.push_back(node);
           }
         }
         this->unlock();
         return nodes;
       }
  };

  typedef uwds::Nodes Nodes;
  typedef boost::shared_ptr<uwds::Nodes> NodesPtr;
  typedef boost::shared_ptr<uwds::Nodes const> NodesConstPtr;
}

#endif
