#ifndef ROBOT_PROVIDER_HPP
#define ROBOT_PROVIDER_HPP

#include "uwds/uwds_client_nodelet.h"
#include "sensor_msgs/JointState.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;
using namespace sensor_msgs;

namespace uwds
{
  class RobotProvider : public UwdsProvider<JointState>
  {
  public:
    RobotProvider():UwdsProvider<JointState> {}

    ~RobotProvider() = default;

    virtual void onInit();

    map<string, string> frame_id_map_;

    string base_frame_id_;

    map<string, Node> link_id_map_;
  };
}

#endif
