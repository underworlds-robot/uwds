#ifndef NODES_HPP
#define NODES_HPP

#include<string>
#include<array>
#include<map>
#include<mutex>
#include "uuid.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "concurrent_container.h"
#include <uwds_msgs/Node.h>

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;


namespace uwds {

  /** @brief
   * The node types enum
   */
  enum NodeType {
    ENTITY = Node::ENTITY,
    MESH = Node::MESH,
    CAMERA = Node::CAMERA
  };
  /** @brief
   * The types names corresponding
   */
  static const array<string,3> NodeTypeName{"entity", "mesh", "camera"};

  /** @brief
   * This class represent the Underworlds nodes container
   */
  class Nodes : public ConcurrentContainer<Node> {

    using ConcurrentContainer::update;

    public:

      Nodes() {
        Node root_node;
        root_node.id = root_id_;
        root_node.name = "root";
        root_node.position.pose.orientation.w = 1.0;
        this->update(root_id_, root_node);
      }

      Nodes(string root_id) {
        root_id_ = root_id;
        Node root_node;
        root_node.id = root_id_;
        root_node.name = "root";
        root_node.position.pose.orientation.w = 1.0;
        this->update(root_id_, root_node);
      }

      ~Nodes() {}
      /** @brief
       * This method update a node (or create one if new)
       *
       * @param node The node to update
       */
      void update(NodePtr node) {
        if(node->name != "root")
        {
          node->children.clear();
          string parent;
          if(!this->has(node->parent)){
            node->parent = root_id_;
            parent = root_id_;
          } else {
            parent = node->parent;
          }
          vector<string> children = (*this)[parent].children;
          this->lock();
          if(std::find(std::begin(children), std::end(children), node->id) == std::end(children))
          {
            (*this)[parent].children.push_back(node->id);
          }
          // normalize quaternion
          tf2::Quaternion q;
          tf2::convert(node->position.pose.orientation , q);
          q.normalize();
          node->position.pose.orientation = tf2::toMsg(q);
          // unlock nodes
          this->unlock();
          // update node
          this->update(node->id, node);
        }
      }

      /** @brief
       * This method update a node (or create one if new)
       *
       * @param node The node to update
       */
      void update(Node node)
      {
        if(node.name != "root")
        {
          node.children.clear();
          string parent;
          if(!this->has(node.parent)){
            node.parent = root_id_;
            parent = root_id_;
          } else {
            parent = node.parent;
          }
          vector<string> children = (*this)[parent].children;
          this->lock();
          if(std::find(std::begin(children), std::end(children), node.id) == std::end(children))
          {
            (*this)[parent].children.push_back(node.id);
          }
          // normalize quaternion
          tf2::Quaternion q;
          tf2::convert(node.position.pose.orientation , q);
          q.normalize();
          node.position.pose.orientation = tf2::toMsg(q);

          this->unlock();
          this->update(node.id, node);
        }
      }

      /** @brief
       * This method update a set of nodes (or create them if new)
       *
       * @param nodes The nodes to update
       */
      void update(const vector<Node> nodes)
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
      void update(const vector<NodePtr> nodes)
      {
        for(const auto& node : nodes)
        {
          update(node);
        }
      }

      string getNodeProperty(const string& node_id, const string& property_name)
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
        * Returns the nodes by property name
        *
        * @param property_name The property name to test
        */
       vector<NodePtr> byProperty(const string& property_name)
       {
         vector<NodePtr> nodes;
         string property;
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
       vector<NodePtr> byProperty(const string& property_name, const string& property_data)
       {
         vector<NodePtr> nodes;
         string property;
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
       vector<NodePtr> byName(const string& name)
       {
         vector<NodePtr> nodes;
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
       vector<NodePtr> byType(const NodeType& type)
       {
         vector<NodePtr> nodes;
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

       string rootID() {return root_id_;}

     protected:

       string root_id_ = NEW_UUID;
  };

  typedef uwds::Nodes Nodes;
  typedef boost::shared_ptr<uwds::Nodes> NodesPtr;
  typedef boost::shared_ptr<uwds::Nodes const> NodesConstPtr;
}

#endif
