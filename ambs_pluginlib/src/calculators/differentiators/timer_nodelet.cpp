#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include "ambs_components/ambs_calculators/differentiators/timer.h"

namespace ambs_calculators
{

class TimerNodelet : public nodelet::Nodelet
{
public:
  TimerNodelet() {}
  ~TimerNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        component_object_.reset(new Timer(nh, name));
        component_object_->init();
      }

private:
    boost::shared_ptr<Timer> component_object_;
};

}  // namespace ambs_calculators

PLUGINLIB_EXPORT_CLASS(ambs_calculators::TimerNodelet,
                       nodelet::Nodelet);
