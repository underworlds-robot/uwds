#ifndef RECONFIGURABLE_CLIENT_HPP
#define RECONFIGURABLE_CLIENT_HPP

#include "uwds/uwds_client_nodelet.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds
{
  class ReconfigurableClient : public UwdsClientNodelet
  {
  public:
    ReconfigurableClient(ClientType type):UwdsClientNodelet(type)
    {
      if(type==PROVIDER)
        throw std::runtime_error("Invalid client type.");
    }
    ~ReconfigurableClient() = default;
    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit();

    virtual void reconfigure(vector<string> inputs);

    virtual bool reconfigureInputs(ReconfigureInputs::Request& req,
                                   ReconfigureInputs::Response& res);

    virtual void onChanges(string world_name, Header header, Invalidations invalidations) = 0;

    virtual void onReconfigure(vector<string> input_worlds) = 0;

    vector<string> inputsWorlds() {return input_worlds_;}

  protected:
    /**
     * A flag to know if use single input.
     */
    bool use_single_input_;

    /** @brief
     * The input world name.
     */
    vector<string> input_worlds_;

    /** @brief
     * The reconfigure service server.
     */
    ros::ServiceServer reconfigure_service_server_;
  };
}

#endif
