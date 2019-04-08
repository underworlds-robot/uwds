#include "uwds/reconfigurable_client.h"

using namespace std;
using namespace std_msgs;
using namespace uwds_msgs;

namespace uwds
{
  void ReconfigurableClient::onInit()
  {
    UwdsClientNodelet::onInit();
    pnh_->param<bool>("use_single_input", use_single_input_, false);
    string default_inputs;
    vector<string> input_worlds;
    pnh_->param<string>("default_inputs", default_inputs, "");
    boost::split(input_worlds, default_inputs, boost::is_any_of(" "));
    reconfigure(input_worlds);
    reconfigure_service_server_ = nh_->advertiseService(ctx_->name()+"/reconfigure_inputs", &ReconfigurableClient::reconfigureInputs, this);
    list_inputs_service_server_ = nh_->advertiseService(ctx_->name()+"/list_inputs", &ReconfigurableClient::listInputs, this);
  }

  void ReconfigurableClient::reconfigure(vector<string> inputs)
  {
    if (inputs.size() > 1 and use_single_input_)
    {
      throw std::runtime_error("Multiple inputs provided while 'use_single_input' activated.");
    }
    ctx_->worlds().close();
    onReconfigure(inputs);
    input_worlds_.clear();

    for(const string input : inputs)
    {
       ctx_->worlds()[input].connect(bind(&ReconfigurableClient::onChanges, this, _1, _2, _3));
       Invalidations invalidations;
       auto& scene = ctx_->worlds()[input].scene();
       auto& timeline = ctx_->worlds()[input].timeline();
       auto& meshes = ctx_->worlds()[input].meshes();
       for (const auto& node : scene.nodes())
          if (node->name !="root")
            invalidations.node_ids_updated.push_back(node->id);
       for (const auto& situation : timeline.situations())
          invalidations.situation_ids_updated.push_back(situation->id);
       for (const auto& mesh : meshes.meshes())
          invalidations.mesh_ids_updated.push_back(mesh->id);
       Header header;
       header.frame_id = global_frame_id_;
       header.stamp = ros::Time::now();
       onChanges(input, header, invalidations);
    }
    input_worlds_ = inputs;
  }

  bool ReconfigurableClient::reconfigureInputs(ReconfigureInputs::Request& req,
                                               ReconfigureInputs::Response& res)
  {
    if(verbose_)NODELET_INFO("[%s::reconfigure] Service '~reconfigure_inputs' requested", ctx_->name().c_str());
    try
    {
      reconfigure(req.inputs);
      res.success = true;
    } catch(const std::exception& e) {
      res.success = false;
      res.error = e.what();
    }
    return true;
  }

  bool ReconfigurableClient::listInputs(List::Request& req,
                                        List::Response& res)
  {
    res.list = inputsWorlds();
    res.success = true;
    return true;
  }
}
