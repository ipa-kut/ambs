#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include "ambs_components/ambs_calculators/differentiators/diff_float_temporal.h"

namespace ambs_calculators
{

class DiffFloatTemporalNodelet : public nodelet::Nodelet
{
public:
  DiffFloatTemporalNodelet() {}
  ~DiffFloatTemporalNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        comp_temporal_.reset(new DiffFloatTemporal(nh, name));
        comp_temporal_->init();
      }

private:
    boost::shared_ptr<DiffFloatTemporal> comp_temporal_;
};

}  // namespace ambs_calculators

PLUGINLIB_EXPORT_CLASS(ambs_calculators::DiffFloatTemporalNodelet,
                       nodelet::Nodelet);
