#ifndef MONITOR_HPP
#define MONITOR_HPP

#include <uwds_msgs/Invalidations.h>
#include <uwds_msgs/Changes.h>
#include <uwds/types/world.h>

using namespace uwds_msgs;

namespace uwds
{
  /** @brief
   * This asbstract class allow to design monitors.
   *
   * Monitors are processes that analyse the geometry and manage situations
   * of a world.
   */
  class Monitor
  {
    public:
      /** @brief
       * The constructor to call by subclass.
       */
      Monitor(const WorldPtr input_world) : input_world_(input_world) {}

      /** @brief
       * The monitor method to implement in subclass.
       */
      virtual Changes monitor(const Invalidations& input_invalidations) = 0;

    private:
      /** @brief
       * The input world.
       */
      WorldPtr input_world_;
  };
}

#endif
