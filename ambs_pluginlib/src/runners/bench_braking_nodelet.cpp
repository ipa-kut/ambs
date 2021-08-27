#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include "ambs_components/ambs_runners/bench_braking.h"

namespace ambs_runners
{

class BenchBrakingNodelet : public nodelet::Nodelet
{
public:
  BenchBrakingNodelet() {}
  ~BenchBrakingNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        class_pointer_.reset(new BenchBraking(nh, name));
        class_pointer_->init();
      }

private:
    boost::shared_ptr<BenchBraking> class_pointer_;
};

}  // namespace ambs_runners

PLUGINLIB_EXPORT_CLASS(ambs_runners::BenchBrakingNodelet,
                       nodelet::Nodelet);
