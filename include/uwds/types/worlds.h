#ifndef WORLDS_HPP
#define WORLDS_HPP

#include<string>
#include<map>

#include "world.h"

namespace uwds {

  typedef std::map<std::string, WorldPtr> WorldMap;
  /** @brief
   * This class represent the worlds container
   */
  class Worlds {
    public:
      Worlds(const MeshesPtr meshes) : meshes_(meshes) {};
      /** @brief
       * The access operator
       *
       * @param name The name of the world to access
       */
      World& operator[](const std::string& name){
        if (worlds_.count(name) == 0) {
          worlds_.emplace(name, boost::make_shared<World>(name, meshes_));
        }
        return *worlds_.at(name);
      }

      /** @brief
       * Return the size of the container
       */
      size_t size() {return worlds_.size();}


      std::vector<Invalidations> applyChanges(const std::vector<ChangesInContextStampedConstPtr>& changes_list)
      {
        std::vector<Invalidations> invalidations;
        for(const auto& changes : changes_list)
        {
          invalidations.push_back(worlds_[changes->ctxt.world]->applyChanges(changes->header, changes->changes));
        }
        return invalidations;
      }

      /** @brief
       * This method reset all the worlds
       */
      void reset()
      {
        worlds_.clear();
      }

    private:
      /** @brief
       * custom Iterator for the nodes,
       * allow to easely explore the map container.
       */
      class Iterator
      {
          WorldMap::iterator it_map_;
        public:
          Iterator(WorldMap::iterator it_map):it_map_(it_map) {}
          WorldPtr operator*() { return (*it_map_).second; }
          Iterator& operator++() { ++it_map_; return *this; }
          bool operator!=(const Iterator& it) const { return it_map_ != it.it_map_; }
      };

      /** @brief
       * custom ConstIterator for the nodes,
       * allow to easely explore the map container.
       */
      class ConstIterator
      {
          WorldMap::const_iterator it_map_;
        public:
          ConstIterator(WorldMap::const_iterator it_map):it_map_(it_map) {}
          WorldConstPtr operator*() const { return (*it_map_).second; }
          ConstIterator& operator++() { ++it_map_; return *this; }
          bool operator!=(const ConstIterator& it) const { return it_map_ != it.it_map_; }
      };

    public:
      /** @brief
       * Returns the begin iterator
       */
      Iterator begin() {return worlds_.begin();}

      /** @brief
       * Returns the begin iterator
       */
      ConstIterator begin() const {return worlds_.begin();}

      /** @brief
       * Returns the end iterator
       */
      Iterator end() {return worlds_.end();}

      /** @brief
       * Returns the end iterator
       */
      ConstIterator end() const {return worlds_.end();}

    private:

      /** @brief
       * The Underworlds meshes.
       */
      MeshesPtr meshes_;

      /** @brief
       * The Worlds by name
       */
      WorldMap worlds_;

  };

  typedef uwds::Worlds Worlds;
  typedef boost::shared_ptr<uwds::Worlds> WorldsPtr;
  typedef boost::shared_ptr<uwds::Worlds const> WorldsConstPtr;
}

#endif
