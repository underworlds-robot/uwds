#ifndef SIMPLE_OBJECT_PROVIDER_HPP
#define SIMPLE_OBJECT_PROVIDER_HPP

#include <uwds/uwds.h>
#include <uwds/uwds_client_nodelet.h>
#include <uwds/tools/model_loader.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace uwds_msgs;
using namespace jsk_recognition_msgs;

namespace uwds
{
  class SimpleObjectProvider : public uwds::UwdsClientNodelet
  {
    public:
      /**@brief
       * The default constructor.
       */
      SimpleObjectProvider(): uwds::UwdsClientNodelet(uwds::PROVIDER) {}

      /**@brief
       * The default destructor
       */
      ~SimpleObjectProvider() = default;

      /** @brief
       * Initialize method. Subclass should call this method
       * in its onInit method.
       */
      virtual void onInit();

    protected:
      /**@brief
       * This method is called when perception data are received.
       */
      void callback(const BoundingBoxConstPtr& msg);

      NodePtr object_node_;

      vector<Mesh> object_meshes_;

      bool use_mesh_ = false;

      string object_class_;

      bool meshes_ever_send_ = false;

      string global_frame_id_;

      string object_name_;

      /**@brief
       * The output world name.
       */
      string output_world_;
      /**@brief
       * Input subscriber for perception data.
       */
      ros::Subscriber input_subscriber_;

      tf::TransformListener tf_listener_;
  };
}

#endif
