#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include "ambs_components/ambs_calculators/differentiators/diff_pose_temporal.h"

namespace ambs_calculators
{

class DiffPoseTemporalNodelet : public nodelet::Nodelet
{
public:
  DiffPoseTemporalNodelet() {}
  ~DiffPoseTemporalNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        comp_temporal_.reset(new DiffPoseTemporal(nh, name));
        comp_temporal_->init();
      }

private:
    boost::shared_ptr<DiffPoseTemporal> comp_temporal_;
};

}  // namespace ambs_calculators

PLUGINLIB_EXPORT_CLASS(ambs_calculators::DiffPoseTemporalNodelet,
                       nodelet::Nodelet);
