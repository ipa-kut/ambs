#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include "ambs_components/ambs_calculators/utils/signal_repeater.h"

namespace ambs_calculators
{

class SignalRepeaterNodelet : public nodelet::Nodelet
{
public:
  SignalRepeaterNodelet() {}
  ~SignalRepeaterNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        class_pointer_.reset(new SignalRepeater(nh, name));
        class_pointer_->init("in_start", "in_stop", "in_reset", "out_done",
                             "in_signal", "out_signal");
      }

private:
    boost::shared_ptr<SignalRepeater> class_pointer_;
};

}  // namespace ambs_calculators

PLUGINLIB_EXPORT_CLASS(ambs_calculators::SignalRepeaterNodelet,
                       nodelet::Nodelet);
