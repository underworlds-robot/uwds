#ifndef WORLD_HPP
#define WORLD_HPP

#include <uwds_msgs/Changes.h>
#include <std_msgs/Header.h>
#include <uwds_msgs/Invalidations.h>
#include "scene.h"
#include "timeline.h"
#include "meshes.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds {

  /** @brief
   * This class represent the world
   */
  class World {
    public:
      /** @brief
       * Default constructor
       */
      World(const string& name, const MeshesPtr meshes)
      {
        name_ = name;
        meshes_ = meshes;
        scene_ = boost::make_shared<Scene>();
        timeline_ = boost::make_shared<Timeline>();
      };

      /** @brief
       * This method update the world.
       */
      Invalidations update(const Changes& changes);

      /** @brief
       * The name accessor
       */
      string name() {return name_;}

      /** @brief
       * The scene accessor
       */
      Scene& scene() {return * scene_;}

      /** @brief
       * The timeline accessor
       */
      Timeline& timeline() {return * timeline_;}

      /** @brief
       * The meshes accessor
       */
      Meshes& meshes() {return * meshes_;}

      /** @brief
       * This method apply changes to the world
       *
       * @param header The header
       * @param changes The changes
       */
      Invalidations applyChanges(const std_msgs::Header& header, const Changes& changes)
      {
        Invalidations invalidations;

        invalidations.node_ids_deleted = scene_->remove(changes.nodes_to_delete);
        invalidations.node_ids_updated = scene_->update(changes.nodes_to_update);

        invalidations.situation_ids_deleted = timeline_->remove(changes.situations_to_delete);
        invalidations.situation_ids_updated = timeline_->update(changes.situations_to_update);

        meshes_->remove(changes.meshes_to_delete);
        invalidations.mesh_ids_deleted = changes.meshes_to_delete;
        for (const auto& mesh : changes.meshes_to_update)
        {
          meshes_->update(mesh);
          invalidations.mesh_ids_updated.push_back(mesh.id);
        }
        return invalidations;
      }

      /** @brief
       * This method reset the world
       */
      void reset()
      {
        scene().reset(scene().rootID());
        timeline().reset(timeline().origin().data);
        meshes().reset();
      }

    private:

      /** @brief
       * The name of the world.
       */
      string name_;

      /** @brief
       * The scene shared pointer.
       */
      ScenePtr scene_;

      /** @brief
       * The timeline shared pointer.
       */
      TimelinePtr timeline_;

      /** @brief
       * The Underworlds meshes.
       */
      MeshesPtr meshes_;
  };

  typedef uwds::World World;
  typedef boost::shared_ptr<uwds::World> WorldPtr;
  typedef boost::shared_ptr<uwds::World const> WorldConstPtr;
}

#endif
