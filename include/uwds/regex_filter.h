#ifndef REGEX_FILTER_HPP
#define REGEX_FILTER_HPP

#include "uwds/reconfigurable_client.h"
#include <regex>

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds
{
  class RegexFilter : public ReconfigurableClient
  {
  public:
    RegexFilter():UwdsClientNodelet(FILTER) {}

    ~RegexFilter() = default;

    virtual void onInit();
  protected:
    virtual void onChanges(string world_name, Header header, Invalidations invalidations);

    virtual void onReconfigure(vector<string> inputs);

    string regex_;
  };
}

#endif
