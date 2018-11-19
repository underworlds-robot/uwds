#ifndef SITUATIONS_HPP
#define SITUATIONS_HPP

#include<ros/ros.h>

#include<string>
#include<array>
#include<map>
#include<mutex>

#include "concurrent_container.h"
#include <uwds_msgs/Situation.h>

using namespace uwds_msgs;

namespace uwds {

  /** @brief
   * The situation type enum
   */
  enum SituationType {
    GENERIC = 0,
    FACT,
    ACTION,
    INTERNAL
  };

  static const std::array<std::string, 4> SituationTypeName{"generic",
                                                           "fact",
                                                           "action",
                                                           "internal"};

  /** @brief
   * This class represent the situations container
   */
  class Situations : public ConcurrentContainer<Situation> {

    using ConcurrentContainer::update;

    public:

      /** @brief
       * This method update a situation (or create one if new)
       *
       * @param situation The situation to update
       */
      void update(const SituationPtr situation) {
        update(situation->id, situation);
      }

      /** @brief
       * This method update a situation (or create one if new)
       *
       * @param situation The situation to update
       */
      void update(const Situation situation) {
        update(situation.id, situation);
      }

      /** @brief
       * This method update a set of situations (or create them if new)
       *
       * @param situations The situations to update
       */
      void update(const std::vector<Situation> situations)
      {
        for(auto& situation : situations)
        {
          update(situation);
        }
      }

      /** @brief
       * This method update a set of situations (or create them if new)
       *
       * @param situations The situations to update
       */
      void update(const std::vector<SituationPtr> situations)
      {
        for(auto& situation : situations)
        {
          update(situation);
        }
      }

      std::string getSituationProperty(const std::string& situation_id, const std::string& property_name)
      {
        this->lock();
        for(const auto& property : (*this)[situation_id].properties)
        {
          if(property.name == property_name)
          {
            this->unlock();
            return property.data;
          }
        }
        this->unlock();
        return "";
      }

      /** @brief
       * Returns the situations by property
       *
       * @param property_name The property name to test
       */
      std::vector<SituationPtr> byProperty(const std::string& property_name)
      {
        std::vector<SituationPtr> situations;
        std::string property;
        this->lock();
        for(const auto situation : *this)
        {
          property = getSituationProperty(situation->id, property_name);
          if(property != "")
            situations.push_back(situation);
        }
        this->unlock();
        return situations;
      }

      /** @brief
       * Returns the situations by property
       *
       * @param property_name The property name to test
       * @param property_data The property data to test
       */
      std::vector<SituationPtr> byProperty(const std::string& property_name, const std::string& property_data)
      {
        std::vector<SituationPtr> situations;
        std::string property;
        this->lock();
        for(const auto& situation : *this)
        {
          std::string property = getSituationProperty(situation->id, property_name);
          if(property == property_data)
            situations.push_back(situation);
        }
        this->unlock();
        return situations;
      }

      /** @brief
       * Returns the situations by name
       *
       * @param description The description to test
       */
      std::vector<SituationPtr> by_description(const std::string& description)
      {
        std::vector<SituationPtr> situations;
        this->lock();
        for(const auto& situation : *this)
        {
          if(situation->description == description)
          {
            situations.push_back(situation);
          }
        }
        this->unlock();
        return situations;
      }

      /** @brief
       * Returns the situations by type
       *
       * @param type The type to test
       */
      std::vector<SituationPtr> by_type(const SituationType& type)
      {
        std::vector<SituationPtr> situations;
        this->lock();
        for(const auto& situation : *this)
        {
          if(situation->type == type)
          {
            situations.push_back(situation);
          }
        }
        this->unlock();
        return situations;
      }
  };

  //for convenience
  typedef boost::shared_ptr<uwds::Situations> SituationsPtr;
  typedef boost::shared_ptr<uwds::Situations const> SituationsConstPtr;
}

#endif
