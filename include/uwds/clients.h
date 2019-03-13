#ifndef UWDS_CLIENTS_HPP
#define UWDS_CLIENTS_HPP

#include "uwds/uwds.h"
#include <nodelet/nodelet.h>

namespace uwds
{
  class UwdsClientNodelet : public nodelet::Nodelet
  {
  public:
    UwdsClient(ClientType type) : type_(type) {}
    ~UwdsClient() = default;
    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit()
    {
      nh_ = boost::make_shared<ros::NodeHandle>(getMTNodeHandle());
      ctx_ = boost::make_shared<UnderworldsProxy>(nh_, getName(), type_);
    }

  private:
    /**
     * The ROS node handle shared pointer.
     */
    boost::shared_ptr<ros::NodeHandle> nh_;

    /** @brief
     * The Underworlds client proxy.
     */
    boost::shared_ptr<UnderworldsProxy> ctx_;
  };

  class ReconfigurableClient : public UwdsClientNodelet
  {
  public:
    ReconfigurableUwdsClient(ClientType type) : type_(type) {}
    ~ReconfigurableUwdsClient() = default;
    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit()
    {
      nh_ = boost::make_shared<ros::NodeHandle>(getMTNodeHandle());
      ctx_ = boost::make_shared<UnderworldsProxy>(nh_, getName(), type_);
      nh_->param<bool>("use_single_input", use_single_input_, false);
    }

    void reconfigure(vector<string> inputs)
    {
      if (inputs > 1 and use_single_input_)
      {
        throw std::runtime_error("Multiple inputs provided.");
      }
      ctx_.worlds().close();
      for(const auto input : inputs)
      {
         ctx_->worlds()[input].connect(&ReconfigurableUwdsClient::onChanges, this);
      }
    }

    virtual bool reconfigureInputs(ReconfigureInputs::Request& req,
                                   ReconfigureInputs::Response& res)
    {
      if(verbose_)NODELET_INFO("[%s] Service '~reconfigure_inputs' requested", ctx_->client.c_str());
      try
      {
        reconfigure(req.inputs);
        res.success = true;
      } catch(const std::exception& e) {
        res.error = e.what();
        res.success = false;
      }
    }

    virtual void onChanges(string world_name, Header header, Invalidations invalidations) = 0;

  private:
    /**
     * A flag to know if use single input.
     */
    bool use_single_input_;

  };

  template<struct Message>
  class UwdsProvider : public UwdsClient
  {
  public:
    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit()
    {
      nh_ = boost::make_shared<ros::NodeHandle>(getMTNodeHandle());
      ctx_ = boost::make_shared<UnderworldsProxy>(nh_, getName(), PROVIDER);
      input_subscriber_ = nh_->subscribe("input", 1, &UwdsProvider::callback, this);
      nh_->param<std::string>("output_world", output_world_, "output");
    }

  private:

    /**@brief
     * This method is called when data are received.
     */
    virtual void callback(const boost::shared_ptr<Message>& msg) = 0;

    /** @brief
     * The output world name.
     */
    std::string output_world_;

    /**@brief
     * Input subscriber for perception data.
     */
    ros::Subscriber input_subscriber_;
  };

  class UwdsReasoner : public UwdsClient
  {
  public:
    UwdsReasoner(ClientType type)
    {
      if(type_==PROVIDER)
        throw std::runtime_error("Invalid client type.");
      type_ = type;
    }

    ~UwdsReasoner() = default;
    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit()
    {
      nh_ = boost::make_shared<ros::NodeHandle>(getMTNodeHandle());
      ctx_ = boost::make_shared<UnderworldsProxy>(nh_, getName(), type_);
      nh_->param<std::string>("input_world", input_world_, "input");
      nh_->param<std::string>("output_world", output_world_, "output");
      ctx_.worlds[input_world_].connect(&UwdsFilter::onChanges);
    }

  private:
    /**@brief
     * This method is called when data are received.
     */
    void onChanges(const std::string& world_name, const Invalidations& invalidations)
    {
      ctx_.worlds[output_world_].update(reasoning(world_name, invalidations));
    }

    virtual Changes& reasoning(const std::string& world_name, const Invalidations& invalidations) = 0;

    ClientType type_;
    /** @brief
     * The input world name.
     */
    std::string input_world_;

    /** @brief
     * The output world name.
     */
    std::string output_world_;
  };

}

#endif
