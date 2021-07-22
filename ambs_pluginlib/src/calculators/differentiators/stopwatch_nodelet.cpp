#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include "ambs_calculators/differentiators/stopwatch.h"

namespace ambs_calculators
{

class StopwatchNodelet : public nodelet::Nodelet
{
public:
  StopwatchNodelet() {}
  ~StopwatchNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        comp_temporal_.reset(new Stopwatch(nh, name));
        comp_temporal_->init();
      }

private:
    boost::shared_ptr<Stopwatch> comp_temporal_;
};

}  // namespace ambs_calculators

PLUGINLIB_EXPORT_CLASS(ambs_calculators::StopwatchNodelet,
                       nodelet::Nodelet);
