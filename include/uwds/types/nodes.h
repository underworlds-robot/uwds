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
    ENTITY = 0,
    MESH,
    CAMERA
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

       /** @brief
        * Returns the nodes by name
        *
        * @param property_name The property name to test
        */
       std::vector<NodePtr> by_property(const std::string& property_name)
       {
         std::vector<NodePtr> nodes;
         this->lock();
         for(const auto node : *this)
         {
           for(const auto property : node->properties)
           {
             if(property.name == property_name)
             {
               nodes.push_back(node);
             }
           }
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
       std::vector<NodePtr> by_property(const std::string& property_name, const std::string& property_data)
       {
         std::vector<NodePtr> nodes;
         this->lock();
         for(const auto node : *this)
         {
           for(const auto property : node->properties)
           {
             if(property.name == property_name && property.data == property_data)
             {
               nodes.push_back(node);
             }
           }
         }
         this->unlock();
         return nodes;
       }

       /** @brief
        * Returns the nodes by name
        *
        * @param name The name to test
        */
       std::vector<NodePtr> by_name(const std::string& name)
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
       std::vector<NodePtr> by_type(const NodeType& type)
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
