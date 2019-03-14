#ifndef SITUATIONS_HPP
#define SITUATIONS_HPP

#include<ros/ros.h>

#include<string>
#include<array>
#include<map>
#include<mutex>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include "concurrent_container.h"
#include <uwds_msgs/Situation.h>

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds {

  /** @brief
   * The situation type enum
   */
  enum SituationType {
    GENERIC = Situation::GENERIC,
    FACT = Situation::FACT,
    ACTION = Situation::ACTION,
    INTERNAL = Situation::INTERNAL
  };
  /** @brief
   * The situation types names corresponding
   */
  static const array<string, 4> SituationTypeName{"generic",
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
      void update(const vector<Situation> situations)
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
      void update(const vector<SituationPtr> situations)
      {
        for(auto& situation : situations)
        {
          update(situation);
        }
      }

      string getSituationProperty(const string& situation_id, const string& property_name)
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
      vector<SituationPtr> byProperty(const string& property_name)
      {
        vector<SituationPtr> situations;
        string property;
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
      vector<SituationPtr> byProperty(const string& property_name, const string& property_data)
      {
        vector<SituationPtr> situations;
        string property;
        this->lock();
        for(const auto& situation : *this)
        {
          string property = getSituationProperty(situation->id, property_name);
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
      vector<SituationPtr> by_description(const string& description)
      {
        vector<SituationPtr> situations;
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
      vector<SituationPtr> by_type(const SituationType& type)
      {
        vector<SituationPtr> situations;
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
