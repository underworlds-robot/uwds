#ifndef SITUATION_HPP
#define SITUATION_HPP

#include <string>
#include <vector>
#include <map>
#include <uwds_msgs/Situation.h>
#include <uwds_msgs/Property.h>

namespace uwds {

  #define NEW_UUID boost::uuids::to_string(boost::uuids::random_generator()())

  /** @brief
   * The node types enum
   */
   enum SituationType {
     GENERIC = uwds_msgs::Situation::GENERIC,
     FACT = uwds_msgs::Situation::FACT,
     ACTION = uwds_msgs::Situation::ACTION,
     INTERNAL = uwds_msgs::Situation::INTERNAL
   };

   static const std::array<std::string, 4> SituationTypeName{"generic",
                                                            "fact",
                                                            "action",
                                                            "internal"};

  struct Situation : public uwds_msgs::Situation {
    private:
      std::string id_;
      std::string name_;
      SituationType type_;
      std::string description_;
      double confidence_;
      ros::Time start;
      ros::Time end;
      std::map<std::string, std::string> properties_;

    public:
      Situation() {
        id_ = NEW_UUID;
        type_ = GENERIC;
      }

      Situation(NodeType type) {
        id_ = NEW_UUID;
        type_ = type;
      }

      Situation(uwds_msgs::Situation situation)
      {
        id_ = situation.id;
        name_ = situation.name;

        switch(situation.type){
          case uwds_msgs::Situation::FACT: {situation.type=FACT; break;}
          case uwds_msgs::Situation::ACTION: {situation.type=ACTION; break;}
          case uwds_msgs::Situation::INTERNAL: {situation.type=INTERNAL; break;}
          default: situation.type=GENERIC;
        }
        description_ = situation.description;
        confidence_ = situation.confidence;

        for (uwds_msgs::Property property : node.properties)
          properties_.emplace(property.name, property.data);
      }

      ~Situation() = default;

      std::string id() const {return(id_);}
      std::string name() const {return(name_);}
      SituationType type() const {return(type_);}
      std::string description() const(return(description_);}
      double confidence() const {return(confidence_);}
      ros::Time start() const {return(start_);}
      ros::Time end() const {return(end_);}

      std::string property(std::string name) const
      {
        if (properties_.count(name) > 0)
          return(properties_.at(name));
        return("");
      }

      void updateProperty(std::string name, std::string data)
      {
        properties_.emplace(name,data);
      }

      void removeProperty(std::string name)
      {
        if (properties_.count(name) > 0)
          properties_.erase(name);
      }

      bool operator==(const Situation& situation) const {return situation.id_ == this->id_;}


  };

  typedef uwds::Situation Situation;
  typedef boost::shared_ptr<uwds::Situation> SituationPtr;
  typedef boost::shared_ptr<uwds::Situation const> SituationConstPtr;
}

#endif
