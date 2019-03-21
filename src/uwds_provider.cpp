#include "uwds/uwds_provider.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds
{
  template<typename Message>
  void UwdsProvider<Message>::onInit()
  {
    UwdsClientNodelet::onInit();
    input_subscriber_ = nh_->subscribe("input", 1, &UwdsProvider<Message>::callback, this);
    pnh_->param<string>("output_world", output_world_, "output");
  }
}
