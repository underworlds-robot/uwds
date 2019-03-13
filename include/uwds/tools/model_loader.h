#ifndef MODEL_LOADER_HPP
#define MODEL_LOADER_HPP

#include <ros/ros.h>
#include<queue>
#include<string>
#include<vector>
#include <uwds_msgs/Node.h>
#include <uwds_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h> // Output data structure
#include <assimp/postprocess.h> // Post processing flags
#include <urdf/model.h>
#include "../types/nodes.h"
#include <tf/tf.h>

using namespace uwds_msgs;

namespace uwds
{
  /** @brief
  * This class allow to load models from files with Assimp
  */
  class ModelLoader
  {
  public:

    ModelLoader()
    {
      importer_ = boost::make_shared<Assimp::Importer>();
    }
    /**@brief
     * This method allow to load a scene.
     *
     * @param filename The filename
     * @param root_node_id The node to parent the scene
     * @param only_meshes If true load only meshes
     * @param meshes_imported The meshes imported
     * @param nodes_imported The nodes imported
     */
    virtual bool loadScene(const std::string& filename,
                           const std::string& root_node_id,
                           const bool only_meshes,
                           std::vector<Mesh>& meshes_imported,
                           std::vector<Node>& nodes_imported);

   /**@brief
    * This method allow to load meshes.
    *
    * @param filename The filename
    * @param scale The scale
    * @param meshes_imported The meshes imported
    * @param aabb The aabb of the meshes
    */
    virtual bool loadMeshes(const std::string& filename,
                            const std::vector<double>& scale,
                            std::vector<Mesh>& meshes_imported,
                            std::vector<double>& aabb);

   /**@brief
    * This method allow to load an entity from an URDF file
    *
    * @param filename The filename
    * @param primitives_folder The primitives_folder path
    * @param meshes_imported The meshes imported
    * @param nodes_imported The nodes imported
    */
    // virtual bool loadURDF(const std::string& filename,
    //                       const std::string& primitives_folder,
    //                       std::vector<Mesh>& meshes_imported,
    //                       std::vector<Node>& nodes_imported);


    private:
      boost::shared_ptr<Assimp::Importer> importer_;
  };
}

#endif
