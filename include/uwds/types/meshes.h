#ifndef MESHES_HPP
#define MESHES_HPP

#include<string>
#include<vector>
#include<map>
#include<mutex>

#include "concurrent_container.h"
#include <uwds_msgs/Mesh.h>

using namespace uwds_msgs;

namespace uwds {

  /** @brief
   * This class represent the Underworlds meshes container
   */
  class Meshes : public ConcurrentContainer<Mesh> {

    using ConcurrentContainer::update;

    public:

      /** @brief
       * This method update a mesh (or create one if new)
       *
       * @param mesh The mesh to update
       */
      void update(const MeshPtr mesh) {
        update(mesh->id, mesh);
      }

      /** @brief
       * This method update a mesh (or create one if new)
       *
       * @param mesh The mesh to update
       */
      void update(const Mesh mesh) {
        update(mesh.id, mesh);
      }

      /** @brief
       * This method update a set of meshes (or create them if new)
       *
       * @param meshes The meshes to update
       */
      void update(const std::vector<Mesh> meshes)
      {
        for(const auto& mesh : meshes)
        {
          update(mesh);
        }
      }

      /** @brief
       * This method update a set of meshes (or create them if new)
       *
       * @param meshes The meshes to update
       */
      void update(const std::vector<MeshPtr> meshes)
      {
        for(const auto& mesh : meshes)
        {
          update(mesh);
        }
      }
  };

  typedef boost::shared_ptr<uwds::Meshes> MeshesPtr;
  typedef boost::shared_ptr<uwds::Meshes const> MeshesConstPtr;
}

#endif
