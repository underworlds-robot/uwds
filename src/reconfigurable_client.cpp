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
    pnh_->param<string>("default_inputs", default_inputs, "");
    pnh_->param<string>("output_world", output_world_, "");
    pnh_->param<string>("output_suffix", output_suffix_, "");
    boost::split(input_worlds_, default_inputs, boost::is_any_of(" "));
    reconfigure(input_worlds_);
    nh_->advertiseService(ctx_->name()+"/reconfigure_inputs", &ReconfigurableClient::reconfigureInputs, this);
  }

  void ReconfigurableClient::reconfigure(vector<string> inputs)
  {
    if (inputs.size() > 1 and use_single_input_)
    {
      throw std::runtime_error("Multiple inputs provided.");
    }
    ctx_->worlds().close();
    onReconfigure(inputs);
    for(const auto input : inputs)
    {
       ctx_->worlds()[input].connect(bind(&ReconfigurableClient::onChanges, this, _1, _2, _3));
       Invalidations invalidations;
       auto& scene = ctx_->worlds()[input].scene();
       auto& timeline = ctx_->worlds()[input].timeline();
       auto& meshes = ctx_->worlds()[input].meshes();
       for (const auto& node : scene.nodes())
          invalidations.node_ids_updated.push_back(node->id);
       for (const auto& situation : timeline.situations())
          invalidations.situation_ids_updated.push_back(situation->id);
       for (const auto& mesh : meshes)
          invalidations.mesh_ids_updated.push_back(mesh->id);
       Header header;
       header.stamp = ros::Time::now();
       onChanges(input, header, invalidations);
    }
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
      res.error = e.what();
      res.success = false;
    }
  }
}
