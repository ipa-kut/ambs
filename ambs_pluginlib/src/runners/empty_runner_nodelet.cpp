#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include "ambs_components/ambs_runners/empty_runner.h"

namespace ambs_runners
{

class EmptyRunnerNodelet : public nodelet::Nodelet
{
public:
  EmptyRunnerNodelet() {}
  ~EmptyRunnerNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        class_pointer_.reset(new EmptyRunner(nh, name));
        class_pointer_->init();
      }

private:
    boost::shared_ptr<EmptyRunner> class_pointer_;
};

}  // namespace ambs_runners

PLUGINLIB_EXPORT_CLASS(ambs_runners::EmptyRunnerNodelet,
                       nodelet::Nodelet);
