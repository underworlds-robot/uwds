#ifndef FILTER_HPP
#define FILTER_HPP

#include <uwds_msgs/Invalidations.h>
#include <uwds_msgs/Changes.h>
#include <uwds/types/world.h>

using namespace uwds_msgs;

namespace uwds
{
  /** @brief
   * This class to design filters.
   *
   * Filters are processes that modify the geometry of an output world based on
   * the updates received in an input world.
   */
  class Filter
  {
    public:
      /** @brief
       * The constructor to call by subclass.
       */
      Filter(const WorldPtr input_world,
             const WorldPtr output_world) : input_world_(input_world),
                                            output_world_(output_world) {}

      /** @brief
       * The filter method to implement in subclass.
       */
      virtual Changes filter(const Invalidations& input_invalidations) = 0;

    private:
      /** @brief
      * The input world.
      */
      WorldPtr input_world_;
      /** @brief
      * The output world.
      */
      WorldPtr output_world_;
  };
}

#endif
