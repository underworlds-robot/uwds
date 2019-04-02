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

  void RegexFilter::onReconfigure(const vector<string>& inputs) {}

  void RegexFilter::onChanges(string world_name, Header header, Invalidations invalidations)
  {
    Changes changes_to_send;
    vector<Node> node_match = ctx_->worlds()[world_name].scene().getNodesByName(regex_);
    for (const auto& node : node_match)
    {
      changes_to_send.nodes_to_update.push_back(node);
    }
    ctx_->worlds()[output_world_].update(header, changes_to_send);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds::RegexFilter, nodelet::Nodelet)
