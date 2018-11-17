#include "uwds/tools/model_loader.h"

#define min(x,y) (x<y?x:y)
#define max(x,y) (y>x?y:x)

using namespace std;


namespace uwds
{
  bool ModelLoader::loadScene(const std::string& filename,
                              const std::string& root_node_id,
                              const bool only_meshes,
                              std::vector<Mesh>& meshes_imported,
                              std::vector<Node>& nodes_imported)
  {
    const aiScene* scene =  importer_->ReadFile(filename.c_str(),
                              aiProcess_Triangulate |
                              aiProcess_GenSmoothNormals |
                              aiProcess_FlipUVs |
                              aiProcess_JoinIdenticalVertices);

    if (scene)
    {
      std::vector<std::string> camera_names;
      std::map<std::string, uwds_msgs::Node> cameras_by_name;
      std::map<std::string, std::string> node_id_by_name;
      if (scene->mNumCameras > 0)
      {
        for (unsigned int nb_camera = 0 ; nb_camera < scene->mNumCameras ; nb_camera++)
        {
          uwds_msgs::Node new_camera;
          new_camera.name = (boost::format("%s") % scene->mCameras[nb_camera]->mName.C_Str()).str();
          uwds_msgs::Property aspect;
          uwds_msgs::Property hfov;
          uwds_msgs::Property clipplanenear;
          uwds_msgs::Property clipplanefar;
          uwds_msgs::Property up;
          uwds_msgs::Property lookat;
          lookat.name = "lookat";
          lookat.data = (boost::format("%f,%f,%f") % (float) scene->mCameras[nb_camera]->mLookAt.x
                                               % (float) scene->mCameras[nb_camera]->mLookAt.y
                                               % (float) scene->mCameras[nb_camera]->mLookAt.z).str();
          up.name = "up";
          up.data = (boost::format("%f,%f,%f") % (float) scene->mCameras[nb_camera]->mUp.x
                                               % (float) scene->mCameras[nb_camera]->mUp.y
                                               % (float) scene->mCameras[nb_camera]->mUp.z).str();
          aspect.name = "aspect";
          aspect.data = (boost::format("%f") % scene->mCameras[nb_camera]->mAspect).str();
          hfov.name = "hfov";
          hfov.data = (boost::format("%f") % scene->mCameras[nb_camera]->mHorizontalFOV).str();
          clipplanenear.name = "clipplanenear";
          clipplanenear.data = (boost::format("%f") % scene->mCameras[nb_camera]->mClipPlaneNear).str();
          clipplanefar.name = "clipplanefar";
          clipplanefar.data = (boost::format("%f") % scene->mCameras[nb_camera]->mClipPlaneFar).str();
          new_camera.properties.push_back(aspect);
          new_camera.properties.push_back(hfov);
          new_camera.properties.push_back(clipplanenear);
          new_camera.properties.push_back(clipplanefar);
          new_camera.properties.push_back(up);
          new_camera.properties.push_back(lookat);
          cameras_by_name.emplace(new_camera.name, new_camera);
        }
      }

      const aiNode* current_node;
      bool mesh_valid;
      bool first_node = true;
      bool first_mesh;
      unsigned int nb_valid_mesh;
      std::queue<const aiNode*> fifo;
      fifo.push(scene->mRootNode);
      //ROS_ERROR("Start exploring file '%s'", filename.c_str());
      while(!fifo.empty()){
        current_node = fifo.front();
        fifo.pop();
        //ROS_ERROR("get node from fifo");
        uwds_msgs::Node new_node;
        new_node.id = NEW_UUID;
        if (current_node != scene->mRootNode) {
          new_node.name = (boost::format("%s") % current_node->mName.C_Str()).str();
        } else {
          new_node.name = "assimp_root";
        }
        new_node.type = ENTITY;
        //ROS_ERROR("Current node <(%s)%s>", new_node.name.c_str(), new_node.id.c_str());
        if(current_node != scene->mRootNode)
        {
          node_id_by_name.emplace(new_node.name, new_node.id);
        } else {
          node_id_by_name.emplace("assimp_root", new_node.id);
        }

        aiVector3t<float> pos;
        aiQuaterniont<float> rot;

        current_node->mTransformation.DecomposeNoScaling(rot, pos);

        if(current_node != scene->mRootNode)
        {
          if(current_node->mParent == scene->mRootNode)
            new_node.parent = root_node_id;
          else
            new_node.parent = node_id_by_name.at((boost::format("%s") % current_node->mParent->mName.C_Str()).str());
        }

        new_node.position.pose.position.x = pos.x;
        new_node.position.pose.position.y = pos.y;
        new_node.position.pose.position.z = pos.z;

        new_node.position.pose.orientation.x = rot.x;
        new_node.position.pose.orientation.y = rot.y;
        new_node.position.pose.orientation.z = rot.z;
        new_node.position.pose.orientation.w = rot.w;

        if(cameras_by_name.count(new_node.name)>0)
        {
          new_node.type = CAMERA;
          new_node.properties = cameras_by_name[new_node.name].properties;
          tf::Quaternion q(new_node.position.pose.orientation.x,
                           new_node.position.pose.orientation.y,
                           new_node.position.pose.orientation.z,
                           new_node.position.pose.orientation.w);
          tf::Quaternion offset;
          offset.setEuler(0,0,0);
          offset = (q + offset);
          tf::quaternionTFToMsg(offset,new_node.position.pose.orientation);
        } else {
          if(current_node->mNumMeshes > 0)
          {
            float x_min = std::numeric_limits<float>::infinity();
            float x_max = 0;
            float y_min = std::numeric_limits<float>::infinity();
            float y_max = 0;
            float z_min = std::numeric_limits<float>::infinity();
            float z_max = 0;

            uwds_msgs::Property new_node_meshes;
            uwds_msgs::Property new_node_aabb;

            new_node_meshes.name = "meshes";
            new_node_aabb.name = "aabb";
            mesh_valid = true;
            nb_valid_mesh = 0;
            first_mesh = true;
            // process here
            for (unsigned int nb_mesh = 0 ; nb_mesh < current_node->mNumMeshes ; nb_mesh++)
            {
              mesh_valid = true;
              const aiMesh* assimp_mesh = scene->mMeshes[current_node->mMeshes[nb_mesh]];
              uwds_msgs::Mesh uwds_mesh;

              uwds_mesh.id = NEW_UUID;

              for(unsigned int nb_vertex = 0 ; nb_vertex < assimp_mesh->mNumVertices ; nb_vertex++)
              {
                const aiVector3D* vertex_pos = &(assimp_mesh->mVertices[nb_vertex]);
                geometry_msgs::Point v;

                v.x = vertex_pos->x;
                v.y = vertex_pos->y;
                v.z = vertex_pos->z;

                x_min = min(x_min,vertex_pos->x);
                y_min = min(y_min,vertex_pos->y);
                z_min = min(z_min,vertex_pos->z);

                x_max = max(x_max,vertex_pos->x);
                y_max = max(y_max,vertex_pos->y);
                z_max = max(z_max,vertex_pos->z);

                uwds_mesh.vertices.push_back(v);

                std_msgs::ColorRGBA c;
                // If the vertex has a color, then we add it
                if (assimp_mesh->HasVertexColors(nb_vertex))
                {
                  const aiColor4D* vertex_color = assimp_mesh->mColors[nb_vertex];
                  c.r = vertex_color->r;
                  c.g = vertex_color->g;
                  c.b = vertex_color->b;
                  c.a = vertex_color->a;
                  uwds_mesh.vertex_colors.push_back(c);
                } else { // Else we look for the materials of the mesh
                  const aiMaterial* material = scene->mMaterials[assimp_mesh->mMaterialIndex];
                  aiColor3D color;
                  material->Get(AI_MATKEY_COLOR_DIFFUSE, color);
                  std_msgs::ColorRGBA c;
                  c.r = color.r;
                  c.g = color.g;
                  c.b = color.b;
                  c.a = 1.0;
                  uwds_mesh.vertex_colors.push_back(c);
                }
              }
              // For each triangle
              for (unsigned int nb_triangle = 0 ; nb_triangle < assimp_mesh->mNumFaces ; nb_triangle++)
              {
                const aiFace& assimp_triangle = assimp_mesh->mFaces[nb_triangle];
                if (assimp_triangle.mNumIndices == 3)
                {
                  uwds_msgs::MeshTriangle t;
                  t.vertex_indices[0] = assimp_triangle.mIndices[0];
                  t.vertex_indices[1] = assimp_triangle.mIndices[1];
                  t.vertex_indices[2] = assimp_triangle.mIndices[2];
                  uwds_mesh.triangles.push_back(t);
                } else {
                  ROS_WARN("Error parsing '%s': Invalid number of vertex indices, skipping the mesh", filename.c_str());
                  mesh_valid = false;
                }
              }
              if (mesh_valid)
              {
                nb_valid_mesh++;
                meshes_imported.push_back(uwds_mesh);
                if (first_mesh)
                {
                  new_node_meshes.data += uwds_mesh.id;
                  first_mesh = false;
                } else {
                  new_node_meshes.data += "," + uwds_mesh.id;
                }
              }
            }

            if (nb_valid_mesh > 0)
            {
              new_node.type = MESH;
              float aabb_x = x_max - x_min;
              float aabb_y = y_max - y_min;
              float aabb_z = z_max - z_min;
              if(aabb_x== 0.0)
                aabb_x = 0.000001;
              if(aabb_y== 0.0)
                aabb_y = 0.000001;
              if(aabb_z== 0.0)
                aabb_z = 0.000001;
              new_node_aabb.data = (boost::format("%f,%f,%f") % aabb_x % aabb_y % aabb_z).str();
              new_node.properties.push_back(new_node_meshes);
              new_node.properties.push_back(new_node_aabb);
            }
          }
          if(current_node != scene->mRootNode)
          {
            nodes_imported.push_back(new_node);
          }
        }
        //ROS_ERROR("%d child to explore ", current_node->mNumChildren);
        if(current_node->mNumChildren > 0)
        {
          for (unsigned int nb_children = 0 ; nb_children <  current_node->mNumChildren ; nb_children++)
          {
            fifo.push(current_node->mChildren[nb_children]);
            //ROS_ERROR("push node to fifo");
          }
        }
      }
      return true;
    } else {
      ROS_ERROR("Loading file '%s' failed : %s", filename.c_str(), importer_->GetErrorString());
      return false;
    }
  }

  bool ModelLoader::loadMeshes(const std::string& filename,
                          const std::vector<double>& scale,
                          std::vector<Mesh>& meshes_imported,
                          std::vector<double>& aabb)
  {
    const aiScene* scene =  importer_->ReadFile(filename.c_str(),
                              aiProcess_Triangulate |
                              aiProcess_GenSmoothNormals |
                              aiProcess_FlipUVs |
                              aiProcess_JoinIdenticalVertices);
    if (scene)
    {
      float x_min = std::numeric_limits<float>::infinity();
      float x_max = 0;
      float y_min = std::numeric_limits<float>::infinity();
      float y_max = 0;
      float z_min = std::numeric_limits<float>::infinity();
      float z_max = 0;
      for (unsigned int nb_mesh = 0 ; nb_mesh < scene->mNumMeshes ; nb_mesh++)
      {
        bool mesh_valid = true;
        const aiMesh* assimp_mesh = scene->mMeshes[nb_mesh];
        uwds_msgs::Mesh uwds_mesh;
        uwds_mesh.id = NEW_UUID;

        for(unsigned int nb_vertex = 0 ; nb_vertex < assimp_mesh->mNumVertices ; nb_vertex++)
        {
          const aiVector3D* vertex_pos = &(assimp_mesh->mVertices[nb_vertex]);
          geometry_msgs::Point v;

          v.x = vertex_pos->x*scale[0];
          v.y = vertex_pos->y*scale[1];
          v.z = vertex_pos->z*scale[2];

          x_min = min(x_min,v.x);
          y_min = min(y_min,v.y);
          z_min = min(z_min,v.z);

          x_max = max(x_max,v.x);
          y_max = max(y_max,v.y);
          z_max = max(z_max,v.z);

          uwds_mesh.vertices.push_back(v);

          std_msgs::ColorRGBA c;
          // If the vertex has a color, then we add it
          if (assimp_mesh->HasVertexColors(nb_vertex))
          {
            const aiColor4D* vertex_color = assimp_mesh->mColors[nb_vertex];
            c.r = vertex_color->r;
            c.g = vertex_color->g;
            c.b = vertex_color->b;
            c.a = vertex_color->a;
            uwds_mesh.vertex_colors.push_back(c);
          } else { // Else we look for the materials of the mesh
            const aiMaterial* material = scene->mMaterials[assimp_mesh->mMaterialIndex];
            aiColor3D color;
            material->Get(AI_MATKEY_COLOR_DIFFUSE, color);
            std_msgs::ColorRGBA c;
            c.r = color.r;
            c.g = color.g;
            c.b = color.b;
            c.a = 1.0;
            uwds_mesh.vertex_colors.push_back(c);
          }
        }
        // For each triangle
        for (unsigned int nb_triangle = 0 ; nb_triangle < assimp_mesh->mNumFaces ; nb_triangle++)
        {
          const aiFace& assimp_triangle = assimp_mesh->mFaces[nb_triangle];
          if (assimp_triangle.mNumIndices == 3)
          {
            uwds_msgs::MeshTriangle t;
            t.vertex_indices[0] = assimp_triangle.mIndices[0];
            t.vertex_indices[1] = assimp_triangle.mIndices[1];
            t.vertex_indices[2] = assimp_triangle.mIndices[2];
            uwds_mesh.triangles.push_back(t);
          } else {
            ROS_WARN("Error parsing '%s': Invalid number of vertex indices, skipping the mesh", filename.c_str());
            mesh_valid = false;
          }
        }
        if(mesh_valid)
        {
          meshes_imported.push_back(uwds_mesh);
        }
      }
      float aabb_x = x_max - x_min;
      float aabb_y = y_max - y_min;
      float aabb_z = z_max - z_min;
      if(aabb_x== 0.0)
        aabb_x = 0.000001;
      if(aabb_y== 0.0)
        aabb_y = 0.000001;
      if(aabb_z== 0.0)
        aabb_z = 0.000001;
      aabb.push_back(aabb_x);
      aabb.push_back(aabb_y);
      aabb.push_back(aabb_z);
    }
  }

  bool ModelLoader::loadURDF(const std::string& filename,
                             const std::string& primitives_folder,
                             std::vector<Mesh>& meshes_imported,
                             std::vector<Node>& nodes_imported)
  {

    urdf::Model model;

    std::string root_link = model.root_link_->name;
    if (!model.initFile(filename))
    {
      ROS_ERROR("Loading file '%s' failed", filename.c_str());
      return false;
    }
    std::queue<urdf::Link> fifo;
    urdf::Link link;
    fifo.push(*model.getRoot());
    do {
      link = fifo.front();
      fifo.pop();
      uwds_msgs::Node new_node;
      new_node.id = NEW_UUID;
      new_node.name = link.name;
      std::vector<double> aabb;
      uwds_msgs::Mesh new_mesh;
      new_mesh.id = NEW_UUID;
      std::vector<uwds_msgs::Mesh> meshes;
      if (link.visual->geometry->type == urdf::Geometry::SPHERE)
      {

        boost::shared_ptr<urdf::Sphere> sphere = boost::dynamic_pointer_cast<urdf::Sphere>(link.visual->geometry);
        std::vector<double> scale;
        scale.push_back(sphere->radius);
        scale.push_back(sphere->radius);
        scale.push_back(sphere->radius);
        loadMeshes(primitives_folder+"sphere.blend",
                   scale,
                   meshes,
                   aabb);
      }
      if (link.visual->geometry->type == urdf::Geometry::BOX)
      {
        boost::shared_ptr<urdf::Box> box = boost::dynamic_pointer_cast<urdf::Box>(link.visual->geometry);
        std::vector<double> scale;
        scale.push_back(box->dim.x);
        scale.push_back(box->dim.y);
        scale.push_back(box->dim.z);
        loadMeshes(primitives_folder+"box.blend",
                   scale,
                   meshes,
                   aabb);
      }
      if (link.visual->geometry->type == urdf::Geometry::CYLINDER)
      {
        boost::shared_ptr<urdf::Cylinder> cylinder = boost::dynamic_pointer_cast<urdf::Cylinder>(link.visual->geometry);
        std::vector<double> scale;
        scale.push_back(cylinder->radius);
        scale.push_back(cylinder->radius);
        scale.push_back(cylinder->length);
        loadMeshes(primitives_folder+"cylinder.blend",
                   scale,
                   meshes,
                   aabb);
      }
      if (link.visual->geometry->type == urdf::Geometry::MESH)
      {
        boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh>(link.visual->geometry);
        std::vector<double> scale;
        scale.push_back(mesh->scale.x);
        scale.push_back(mesh->scale.y);
        scale.push_back(mesh->scale.z);
        loadMeshes(mesh->filename,
                   scale,
                   meshes,
                   aabb);
      }

      for (const auto& child_ptr : link.child_links)
      {
        fifo.push(*child_ptr);
      }
    } while (!fifo.empty());

    for (const auto& link_pair : model.links_)
    {
      uwds_msgs::Node new_node;
      new_node.name = link_pair.first;
      new_node.id = NEW_UUID;
    }
    return true;
  }
}
