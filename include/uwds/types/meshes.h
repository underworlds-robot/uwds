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
      std::vector<std::string> update(const std::vector<Mesh> meshes)
      {
        std::vector<std::string> mesh_ids;
        for(const auto& mesh : meshes)
        {
          mesh_ids.push_back(mesh.id);
          update(mesh);
        }
        return mesh_ids;
      }

      /** @brief
       * This method update a set of meshes (or create them if new)
       *
       * @param meshes The meshes to update
       */
      std::vector<std::string> update(const std::vector<MeshPtr> meshes)
      {
        std::vector<std::string> mesh_ids;
        for(const auto& mesh : meshes)
        {
          mesh_ids.push_back(mesh->id);
          update(mesh);
        }
        return mesh_ids;
      }
  };

  typedef boost::shared_ptr<uwds::Meshes> MeshesPtr;
  typedef boost::shared_ptr<uwds::Meshes const> MeshesConstPtr;
}

#endif
