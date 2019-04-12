#ifndef CONCURRENT_CONTAINER_HPP
#define CONCURRENT_CONTAINER_HPP

#include<string>
#include<vector>
#include<map>
#include<mutex>

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds {

  /** @brief
   * This class allow to store thead-safe data indexed by a unique key.
   */
  template<class Element>
  class ConcurrentContainer
  {
    typedef boost::shared_ptr<Element> ElementPtr;
    typedef boost::shared_ptr<Element const> ElementConstPtr;
    typedef map<string, ElementPtr> ElementMap;

    public:

      /** @brief
       * This method update an element (or create one if new).
       *
       * @param id The id of the element
       * @param element The element to update
       */
      void update(const string id, const ElementPtr element)
      {
        if(map_.count(id)>0)
        {
          map_.at(id) = element;
        } else {
          lock();
          map_.emplace(id, element);
          unlock();
        }
      }

      /** @brief
       * This method update an element (or create one if new).
       *
       * @param id The id of the element
       * @param element The element to update
       */
      void update(const string id, const Element element)
      {
        if(map_.count(id)>0)
        {
          map_.at(id) = boost::make_shared<Element>(element);
        } else {
          lock();
          map_.emplace(id, boost::make_shared<Element>(element));
          unlock();
        }
      }

      /** @brief
       * This method remove an element.
       *
       * @param id The ID of the element to remove
       */
      virtual void remove(const string id)
      {
        if(map_.count(id)>0){
          lock();
          map_.erase(id);
          unlock();
        }
      }

      /** @brief
       * This method remove a set of element.
       *
       * @param ids The IDs of the elements to remove
       */
      virtual void remove(const vector<string> ids)
      {
        lock();
        for(const string id : ids)
          if(map_.count(id)>0)
            map_.erase(id);
        unlock();
      }

      /** @brief
       * The access operator.
       *
       * @param id The ID of the element to access
       */
      Element& operator[](const string& id) {return *map_.at(id);}

      /** @brief
       * This method test if an element exist.
       *
       * @param id The ID of the element to test
       */
      bool has(string id) {return(map_.count(id)>0);}

      /** @brief
       * This method check if the container is empty.
       */
      bool empty() {return(map_.empty());}

      /** @brief
       * Returns the size on the container.
       */
      size_t size() {return map_.size();}

      /** @brief
       * Reset the container.
       */
      void reset() {map_.clear();}

      /** @brief
       * Lock the container.
       */
      void lock() {mutex_.lock();}

      /** @brief
       * Unlock the container.
       */
      void unlock() {mutex_.unlock();}

    public:
      /** @brief
       * custom Iterator for the nodes,
       * allow to easely explore the map container.
       */
      class Iterator
      {
          typename ElementMap::iterator it_map_;
        public:
          Iterator(typename ElementMap::iterator it_map):it_map_(it_map) {}
          ElementPtr operator*() { return (*it_map_).second; }
          Iterator& operator++() { ++it_map_; return *this; }
          bool operator!=(const Iterator& it) const { return it_map_ != it.it_map_; }
      };

      /** @brief
       * custom ConstIterator for the nodes,
       * allow to easely explore the map container.
       */
      class ConstIterator
      {
          typename ElementMap::const_iterator it_map_;
        public:
          ConstIterator(typename ElementMap::const_iterator it_map):it_map_(it_map) {}
          ElementConstPtr operator*() const { return (*it_map_).second; }
          ConstIterator& operator++() { ++it_map_; return *this; }
          bool operator!=(const ConstIterator& it) const { return it_map_ != it.it_map_; }
      };

    public:
      /** @brief
       * Returns the begin iterator.
       */
      Iterator begin() {return map_.begin();}

      /** @brief
       * Returns the begin iterator.
       */
      ConstIterator begin() const {return map_.begin();}

      /** @brief
       * Returns the end iterator.
       */
      Iterator end() {return map_.end();}

      /** @brief
       * Returns the end iterator.
       */
      ConstIterator end() const {return map_.end();}

    private:
      /** @brief
       * The map containing the elements by ID.
       */
      ElementMap map_;

      /** @brief
       * The mutex of the container.
       */
      mutex mutex_;
  };
}

#endif
