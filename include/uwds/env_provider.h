#ifndef ENV_PROVIDER_HPP
#define ENV_PROVIDER_HPP

#include "uwds/uwds_client_nodelet.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds
{
  class EnvProvider : public UwdsClientNodelet
  {
  public:
    EnvProvider():UwdsClientNodelet(PROVIDER) {}

    ~EnvProvider() = default;

    virtual void onInit();
  };
}

#endif
