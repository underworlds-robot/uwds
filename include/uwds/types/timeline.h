#ifndef TIMELINE_HPP
#define TIMELINE_HPP

#include<ros/ros.h>
#include "situations.h"
#include <functional>
#include <regex>

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;


namespace uwds {

  using onEventFcn = void(string, string);

  enum EventMode {
    RISING = 0,
    FALLING
  };

  struct Event {
    string regex;
    EventMode mode;
    function<onEventFcn> callback;
  };

  typedef ConcurrentContainer<Event> EventRegister;
  typedef boost::shared_ptr<EventRegister> EventRegisterPtr;

  /** @brief
   * This class represent the timeline
   */
  class Timeline {
    public:
      /** @brief
      * The default constructor
      */
      Timeline() {
        origin_.data = ros::Time::now();
        situations_ = boost::make_shared<Situations>();
        event_register_ = boost::make_shared<EventRegister>();
      }

      /** @brief
       * Copy destructor.
       */
      Timeline(const Timeline& timeline) = default;

      /** @brief
       * Move destructor.
       */
      Timeline(Timeline&& timeline) = default;

      /** @brief
       * Default destructor.
       */
      ~Timeline() = default;

      /** @brief
       * This method update a situation (or create one if new)
       *
       * @param situation The situation to update
       * @return The situation id updated
       */
      std::string update(const SituationPtr situation)
      {
        situations_->update(situation);
        //evaluate(*situation);
        return situation->id;
      }

      /** @brief
       * This method update a situation (or create one if new)
       *
       * @param situation The situation to update
       * @return The situation id updated
       */
      std::string update(const Situation situation)
      {
        situations_->update(situation);
        //evaluate(situation);
        return situation.id;
      }

      /** @brief
       * This method update a set of situations (or create them if new)
       *
       * @param situations The situations to update
       * @return The situation ids updated
       */
      std::vector<std::string> update(const std::vector<Situation> situations)
      {
        std::vector<std::string> situation_ids;
        for (const auto& situation : situations)
          situation_ids.push_back(this->update(situation));
        return situation_ids;
      }

      /** @brief
       * This method update a set of situations (or create them if new)
       *
       * @param situations The situations to update
       * @return The situation ids updated
       */
      std::vector<std::string> update(const std::vector<SituationPtr> situations)
      {
        std::vector<std::string> situation_ids;
        for (const auto& situation : situations)
          situation_ids.push_back(this->update(situation));
        return situation_ids;
      }

      bool evaluate(const Situation& situation)
      {
        string key = "";
        bool triggered = false;
        for (const auto event : *event_register_)
        {
          if(regex_match(situation.description, regex(event->regex)))
          {
            if(event->mode == FALLING)
            {
              if(situation.end.data != ros::Time())
              {
                event->callback(event->regex, situation.id);
                key = event->regex;
                triggered = true;
              }
            } else {
              event->callback(event->regex, situation.id);
              key = event->regex;
              triggered = true;
            }
          }
        }
        if (triggered) event_register_->remove(key);
        return triggered;
      }

      /** @brief
       * This method remove a situation
       *
       * @param id The situation id to delete
       * @return The situation id deleted
       */
      std::string remove(const std::string id)
      {
        situations_->remove(id);
        return id;
      }

      /** @brief
       * This method remove a set of situations
       *
       * @param ids The situation ids to delete
       * @return The situation ids deleted
       */
      std::vector<std::string> remove(const std::vector<std::string> ids)
      {
        situations_->remove(ids);
        return ids;
      }

      /** @brief
      * This method reset the timeline with the given origin
      *
      * @param origin The origin of the timeline
      * @return The situation ids deleted
      */
      std::vector<std::string> reset(ros::Time origin)
      {
        std::vector<std::string> situation_ids;
        for (const auto& situation : situations())
          situation_ids.push_back(situation->id);
        situations_->reset();
        origin_.data = origin;
        return situation_ids;
      }

      void connect(Event event) {event_register_->update(event.regex, event);}

      /** @brief
       * Lock the timeline.
       */
      void lock() {situations_->lock();}

      /** @brief
       * Unlock the timeline.
       */
      void unlock() {situations_->unlock();}

      /** @brief
       * Returns the size on the timeline.
       */
      uint size()
      {
        return situations_->size();
      }

      /** @brief
      * The origin getter
      */
      std_msgs::Time origin() const {return origin_;}

      /** @brief
      * The situations container getter
      */
      Situations& situations() {return *situations_;}

    private:
      /** @brief
      * The timeline origin
      */
      std_msgs::Time origin_;

      /** @brief
      * The situations container pointer
      */
      SituationsPtr situations_;

      /** @brief
      * The event register shared pointer.
      */
      EventRegisterPtr event_register_;
  };

  typedef boost::shared_ptr<uwds::Timeline> TimelinePtr;
  typedef boost::shared_ptr<uwds::Timeline const> TimelineConstPtr;
}

#endif
