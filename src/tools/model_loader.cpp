#include "uwds/tools/model_loader.h"

#define min(x,y) (x<y?x:y)
#define max(x,y) (y>x?y:x)

using namespace std;
using namespace uwds_msgs;

#if URDFDOM_HEADERS_MAJOR_VERSION == 0 && URDFDOM_HEADERS_MINOR_VERSION <= 4
  template <class T, class U>
  boost::shared_ptr<T> my_pointer_cast(const boost::shared_ptr<U> &ptr){
    return boost::dynamic_pointer_cast<T>(ptr);
  }
#else
  template <class T, class U>
  std::shared_ptr<T> my_pointer_cast(const std::shared_ptr<U> &ptr){
    return std::dynamic_pointer_cast<T>(ptr);
  }
#endif

namespace uwds
{
  enum JointType {
    REVOLUTE = urdf::Joint::REVOLUTE,
    CONTINUOUS = urdf::Joint::CONTINUOUS,
    PRISMATIC = urdf::Joint::PRISMATIC,
    FLOATING = urdf::Joint::FLOATING,
    PLANAR = urdf::Joint::PLANAR,
    FIXED = urdf::Joint::FIXED
    };

  static const std::array<std::string,6> JointTypeName{"revolute",
                                                       "continuous",
                                                       "prismatic",
                                                       "floating",
                                                       "planar",
                                                       "fixed"};

  bool ModelLoader::loadScene(const string& filename,
                              const string& root_node_id,
                              const bool only_meshes,
                              vector<Mesh>& meshes_imported,
                              vector<Node>& nodes_imported)
  {
    const aiScene* scene =  importer_->ReadFile(filename.c_str(),
                              aiProcess_Triangulate |
                              aiProcess_GenSmoothNormals |
                              aiProcess_FlipUVs |
                              aiProcess_JoinIdenticalVertices);

    if (scene)
    {
      vector<string> camera_names;
      map<string, uwds_msgs::Node> cameras_by_name;
      map<string, string> node_id_by_name;
      if (scene->mNumCameras > 0)
      {
        for (unsigned int nb_camera = 0 ; nb_camera < scene->mNumCameras ; nb_camera++)
        {
          uwds_msgs::Node new_camera;
          new_camera.name = string(scene->mCameras[nb_camera]->mName.C_Str());
          uwds_msgs::Property aspect;
          uwds_msgs::Property hfov;
          uwds_msgs::Property clipplanenear;
          uwds_msgs::Property clipplanefar;
          uwds_msgs::Property up;
          uwds_msgs::Property lookat;
          lookat.name = "lookat";
          lookat.data = to_string((float) scene->mCameras[nb_camera]->mLookAt.x)+","
                        + to_string((float) scene->mCameras[nb_camera]->mLookAt.y)+","
                        + to_string((float) scene->mCameras[nb_camera]->mLookAt.z);

          up.name = "up";
          up.data = to_string((float) scene->mCameras[nb_camera]->mUp.x)+","
                    + to_string((float) scene->mCameras[nb_camera]->mUp.y)+","
                    + to_string((float) scene->mCameras[nb_camera]->mUp.z);

          aspect.name = "aspect";
          aspect.data = to_string(scene->mCameras[nb_camera]->mAspect);
          hfov.name = "hfov";
          hfov.data = to_string(scene->mCameras[nb_camera]->mHorizontalFOV);
          clipplanenear.name = "clipplanenear";
          clipplanenear.data = to_string(scene->mCameras[nb_camera]->mClipPlaneNear);
          clipplanefar.name = "clipplanefar";
          clipplanefar.data = to_string(scene->mCameras[nb_camera]->mClipPlaneFar);
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
      queue<const aiNode*> fifo;
      fifo.push(scene->mRootNode);
      //ROS_ERROR("Start exploring file '%s'", filename.c_str());
      while(!fifo.empty()){
        current_node = fifo.front();
        fifo.pop();
        //ROS_ERROR("get node from fifo");
        uwds_msgs::Node new_node;
        new_node.id = NEW_UUID;
        if (current_node != scene->mRootNode) {
          new_node.name = string(current_node->mName.C_Str());
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
            new_node.parent = node_id_by_name.at(string(current_node->mParent->mName.C_Str()));
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
            float x_min = numeric_limits<float>::infinity();
            float x_max = 0;
            float y_min = numeric_limits<float>::infinity();
            float y_max = 0;
            float z_min = numeric_limits<float>::infinity();
            float z_max = 0;

            uwds_msgs::Property new_node_meshes;
            uwds_msgs::Property new_node_aabb;
            uwds_msgs::Property new_node_class;

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
                  shape_msgs::MeshTriangle t;
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
              new_node_aabb.data = to_string(aabb_x)+","+to_string(aabb_y)+","+to_string(aabb_z);
              new_node.properties.push_back(new_node_meshes);
              new_node.properties.push_back(new_node_aabb);
              new_node_class.name = "class";
              new_node_class.data = "Obstacle";
              new_node.properties.push_back(new_node_class);
            }
          }
          if(current_node != scene->mRootNode)
          {
            nodes_imported.push_back(new_node);
          }
        }
        if(current_node->mNumChildren > 0)
        {
          for (unsigned int nb_children = 0 ; nb_children <  current_node->mNumChildren ; nb_children++)
          {
            fifo.push(current_node->mChildren[nb_children]);
          }
        }
      }
      return true;
    } else {
      ROS_ERROR("Loading file '%s' failed : %s", filename.c_str(), importer_->GetErrorString());
      return false;
    }
  }

  bool ModelLoader::loadMeshes(const string& filename,
                          const vector<double>& scale,
                          vector<Mesh>& meshes_imported,
                          vector<double>& aabb)
  {
    const aiScene* scene =  importer_->ReadFile(filename.c_str(),
                              aiProcess_Triangulate |
                              aiProcess_GenSmoothNormals |
                              aiProcess_FlipUVs |
                              aiProcess_JoinIdenticalVertices);
    if (scene)
    {
      float x_min = numeric_limits<float>::infinity();
      float x_max = 0;
      float y_min = numeric_limits<float>::infinity();
      float y_max = 0;
      float z_min = numeric_limits<float>::infinity();
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
            shape_msgs::MeshTriangle t;
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
      return true;
    }
    return false;
  }

  bool ModelLoader::loadURDF(const string& filename,
                             const string& primitives_folder,
                             const string& root_id,
                             vector<Mesh>& meshes_imported,
                             vector<Node>& nodes_imported)
  {
    urdf::Model model;
    map<string, string> node_id_by_frame;
    if (!model.initFile(filename))
    {
      ROS_ERROR("Loading file '%s' failed", filename.c_str());
      return false;
    }
    for(const auto& joint_pair : model.joints_)
    {
      uwds_msgs::Node new_node;
      new_node.id = NEW_UUID;
      new_node.type = ENTITY;
      new_node.name = joint_pair.first;
      new_node.position.pose.position.x = joint_pair.second->parent_to_joint_origin_transform.position.x;
      new_node.position.pose.position.y = joint_pair.second->parent_to_joint_origin_transform.position.y;
      new_node.position.pose.position.z = joint_pair.second->parent_to_joint_origin_transform.position.z;
      new_node.position.pose.orientation.x = joint_pair.second->parent_to_joint_origin_transform.rotation.x;
      new_node.position.pose.orientation.y = joint_pair.second->parent_to_joint_origin_transform.rotation.y;
      new_node.position.pose.orientation.z = joint_pair.second->parent_to_joint_origin_transform.rotation.z;
      new_node.position.pose.orientation.w = joint_pair.second->parent_to_joint_origin_transform.rotation.w;
      Property axis_property;
      axis_property.name = "axis";
      axis_property.data = to_string(joint_pair.second->axis.x) + "," + to_string(joint_pair.second->axis.y) + "," + to_string(joint_pair.second->axis.z);
      Property joint_type_property;
      joint_type_property.name = "joint";
      switch (joint_pair.second->type) {
        case REVOLUTE: joint_type_property.data = JointTypeName[REVOLUTE]; break;
        case CONTINUOUS: joint_type_property.data = JointTypeName[CONTINUOUS]; break;
        case PRISMATIC: joint_type_property.data = JointTypeName[PRISMATIC]; break;
        case FLOATING: joint_type_property.data = JointTypeName[FLOATING]; break;
        case PLANAR: joint_type_property.data = JointTypeName[PLANAR]; break;
        case FIXED: joint_type_property.data = JointTypeName[FIXED]; break;
        default : joint_type_property.data = "unknown";
      }
      nodes_imported.push_back(new_node);
      node_id_by_frame.emplace(new_node.name, new_node.id);
    }

    queue<urdf::Link> fifo;
    urdf::Link link;
    //string parent = root_id;
    fifo.push(*model.getRoot());
    do {
      link = fifo.front();
      fifo.pop();
      uwds_msgs::Node new_node;
      new_node.id = NEW_UUID;
      new_node.type = MESH;
      new_node.name = link.name;
      vector<double> aabb;
      vector<uwds_msgs::Mesh> meshes;
      node_id_by_frame.emplace(new_node.name, new_node.id);

      new_node.position.pose.position.x = link.visual->origin.position.x;
      new_node.position.pose.position.y = link.visual->origin.position.y;
      new_node.position.pose.position.z = link.visual->origin.position.z;
      new_node.position.pose.orientation.x = link.visual->origin.rotation.x;
      new_node.position.pose.orientation.y = link.visual->origin.rotation.y;
      new_node.position.pose.orientation.z = link.visual->origin.rotation.z;
      new_node.position.pose.orientation.w = link.visual->origin.rotation.w;

      if (link.visual->geometry->type == urdf::Geometry::SPHERE)
      {
        urdf::SphereSharedPtr sphere = my_pointer_cast<urdf::Sphere>(link.visual->geometry);
        vector<double> scale;
        scale.push_back(sphere->radius);
        scale.push_back(sphere->radius);
        scale.push_back(sphere->radius);
        loadMeshes(primitives_folder+"/3ds/sphere.3ds",
                   scale,
                   meshes,
                   aabb);
      }
      if (link.visual->geometry->type == urdf::Geometry::BOX)
      {
        urdf::BoxSharedPtr box = my_pointer_cast<urdf::Box>(link.visual->geometry);
        vector<double> scale;
        scale.push_back(box->dim.x);
        scale.push_back(box->dim.y);
        scale.push_back(box->dim.z);
        loadMeshes(primitives_folder+"/3ds/box.3ds",
                   scale,
                   meshes,
                   aabb);
      }
      if (link.visual->geometry->type == urdf::Geometry::CYLINDER)
      {
        urdf::CylinderSharedPtr cylinder = my_pointer_cast<urdf::Cylinder>(link.visual->geometry);
        vector<double> scale;
        scale.push_back(cylinder->radius);
        scale.push_back(cylinder->radius);
        scale.push_back(cylinder->length);
        loadMeshes(primitives_folder+"/3ds/cylinder.3ds",
                   scale,
                   meshes,
                   aabb);
      }
      if (link.visual->geometry->type == urdf::Geometry::MESH)
      {
        urdf::MeshSharedPtr mesh = my_pointer_cast<urdf::Mesh>(link.visual->geometry);
        vector<double> scale;
        scale.push_back(mesh->scale.x);
        scale.push_back(mesh->scale.y);
        scale.push_back(mesh->scale.z);
        loadMeshes(mesh->filename,
                   scale,
                   meshes,
                   aabb);
      }
      Property meshes_property;
      meshes_property.name = "meshes";
      bool first_mesh = true;
      for(const auto& mesh : meshes)
      {
        meshes_imported.push_back(mesh);
        if(first_mesh)
          meshes_property.data += mesh.id;
        else
          meshes_property.data += "," + mesh.id;
      }
      new_node.properties.push_back(meshes_property);
      Property aabb_property;
      aabb_property.name = "aabb";
      aabb_property.data = to_string(aabb[0])+","+to_string(aabb[1])+","+to_string(aabb[2]);
      new_node.properties.push_back(aabb_property);
      Property class_property;
      class_property.name = "class";
      class_property.data = "BodyPart";
      if(new_node.name == "head")
        class_property.data = "Head";
      if(new_node.name =="gripper" || new_node.name =="r_gripper" || new_node.name =="l_gripper")
        class_property.data = "Hand";
      if(new_node.name == "torso")
        class_property.data = "Torso";
      new_node.properties.push_back(class_property);
      nodes_imported.push_back(new_node);
      for (const auto& child_ptr : link.child_links)
      {
        fifo.push(*child_ptr);
      }
      for(auto& node : nodes_imported)
      {
        if(node.name == model.getRoot()->name)
          new_node.parent = root_id;
        else{
          if(node.type == MESH)
          {
            if(node_id_by_frame.count(model.getLink(node.name)->parent_joint->name) != 0)
              node.parent = node_id_by_frame.at(model.getLink(node.name)->parent_joint->name);
          }
          else {
            if(node_id_by_frame.count(model.getJoint(node.name)->parent_link_name) != 0)
              node.parent = node_id_by_frame.at(model.getJoint(node.name)->parent_link_name);
          }
        }
      }
    } while (!fifo.empty());
    return true;
  }
}
