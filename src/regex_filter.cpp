#include "uwds/regex_filter.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds
{
  void RegexFilter::onInit()
  {
    getMTPrivateNodeHandle().param<string>("default_regex", regex_, "table|yellow|orange")
    ReconfigurableClient::onInit();
  }

  void RegexFilter::onReconfigure(const vector<string>& inputs)
  {
    //TODO reset world
  }

  void RegexFilter::onChanges(string world_name, Header header, Invalidations invalidations)
  {

  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds::RegexFilter, nodelet::Nodelet)
