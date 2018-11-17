#ifndef TIMELINE_HPP
#define TIMELINE_HPP

#include<ros/ros.h>

#include "situations.h"

namespace uwds {

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
       */
      void update(const SituationPtr situation)
      {
        situations_->update(situation);
      }

      /** @brief
       * This method update a situation (or create one if new)
       *
       * @param situation The situation to update
       */
      void update(const Situation situation)
      {
        situations_->update(situation);
      }

      /** @brief
       * This method update a set of situations (or create them if new)
       *
       * @param situations The situations to update
       */
      void update(const std::vector<Situation> situations)
      {
        situations_->update(situations);
      }

      /** @brief
       * This method update a set of situations (or create them if new)
       *
       * @param situations The situations to update
       */
      void update(const std::vector<SituationPtr> situations)
      {
        situations_->update(situations);
      }

      /** @brief
       * This method remove a situation
       *
       * @param id The situation id to delete
       */
      void remove(const std::string id)
      {
        situations_->remove(id);
      }

      /** @brief
       * This method remove a set of situations
       *
       * @param ids The situation ids to delete
       */
      void remove(const std::vector<std::string> ids)
      {
        situations_->remove(ids);
      }

      /** @brief
      * This method reset the timeline with the given origin
      *
      * @param origin The origin of the timeline
      */
      void reset(ros::Time origin)
      {
        situations_->reset();
        origin_.data = origin;
      }

      /** @brief
      * This method reset the timeline
      */
      void reset()
      {
        situations_->reset();
        origin_.data = ros::Time::now();
      }

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
  };

  typedef boost::shared_ptr<uwds::Timeline> TimelinePtr;
  typedef boost::shared_ptr<uwds::Timeline const> TimelineConstPtr;
}

#endif
