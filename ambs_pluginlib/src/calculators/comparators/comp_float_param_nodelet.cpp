#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include "ambs_components/ambs_calculators/comparators/float_param_comparator.h"

namespace ambs_calculators
{

class CompFloatParamNodelet : public nodelet::Nodelet
{
public:
  CompFloatParamNodelet() {}
  ~CompFloatParamNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        class_pointer_.reset(new CompFloatParam(nh, name));
        class_pointer_->init("in_start", "in_stop", "in_reset", "out_done",
                             "out_comparison", "in_float");
      }

private:
    boost::shared_ptr<CompFloatParam> class_pointer_;
};

}  // namespace ambs_calculators

PLUGINLIB_EXPORT_CLASS(ambs_calculators::CompFloatParamNodelet,
                       nodelet::Nodelet);
