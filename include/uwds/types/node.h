#ifndef NODE_HPP
#define NODE_HPP

#include <string>
#include <set>
#include <uwds_msgs/Node.h>
#include <uwds_msgs/Property.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/AccelWithCovariance.h>

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

  struct Node {
    private:
      std::string id_;
      std::string name_;
      NodeType type_;
      std::string parent_;
      std::vector<std::string> children_;
      geometry_msgs::PoseWithCovariance position_;
      geometry_msgs::TwistWithCovariance velocity_;
      geometry_msgs::AccelWithCovariance acceleration_;
      std::map<std::string, std::string> properties_;
    public:

      Node()
      {
        id_ = NEW_UUID;
        type_ = ENTITY;
      }

      Node(NodeType type)
      {
        id_ = NEW_UUID;
        type_ = type;
      }

      Node(uwds_msgs::Node node)
      {
        id_ = node.id;
        name_ = node.name;

        switch(node.type){
          case uwds_msgs::Node::MESH: {node.type=MESH; break;}
          case uwds_msgs::Node::CAMERA: {node.type=CAMERA; break;}
          default: node.type=ENTITY;
        }
        parent_ = node.parent;
        children_ = node.children;

        position_ = node.position;
        velocity_ = node.velocity;
        acceleration_ = node.acceleration;

        for (uwds_msgs::Property property : node.properties)
          properties_.emplace(property.name, property.data);
      }

      ~Node() = default;

      std::string id() const {return(id_);}
      std::string name() const {return(name_);}
      NodeType type() const {return(type_);}
      std::string parent() const {return(parent_);}
      std::vector<std::string> children() const {return(children_);}
      geometry_msgs::PoseWithCovariance position() const {return(position_);}
      geometry_msgs::TwistWithCovariance velocity() const {return(velocity_);}
      geometry_msgs::AccelWithCovariance acceleration() const {return(acceleration_);}
      std::string property(std::string name) const
      {
        if (properties_.count(name) > 0)
          return(properties_.at(name));
        return("");
      }

      bool operator==(const Node& node) const {return node.id_ == this->id_;}

      void setId(std::string id) {id_=id;}
      void setName(std::string name) {name_=name;}
      void setType(NodeType type) {type_=type;}
      void setParent(std::string parent) {parent_=parent;}

      void addChild(std::string child)
      {
        if (std::find(children_.begin(), children_.end(), child) == children_.end())
          children_.push_back(child);
      }
      void removeChild(std::string child)
      {
        for (auto it = children_.begin(); it != children_.end();)
        {
          if(*it==child) {
            it = children_.erase(it);
          } else{++it;}
        }
      }

      void setPosition(geometry_msgs::PoseWithCovariance position) {position_ = position;}
      void setVelocity(geometry_msgs::TwistWithCovariance velocity) {velocity_ = velocity;}
      void setAcceleration(geometry_msgs::AccelWithCovariance acceleration) {acceleration_ = acceleration;}

      void updateProperty(std::string name, std::string data)
      {
        properties_.emplace(name,data);
      }

      void removeProperty(std::string name)
      {
        if (properties_.count(name) > 0)
          properties_.erase(name);
      }

    };

  typedef uwds::Node Node;
  typedef boost::shared_ptr<uwds::Node> NodePtr;
  typedef boost::shared_ptr<uwds::Node const> NodeConstPtr;

}

#endif
