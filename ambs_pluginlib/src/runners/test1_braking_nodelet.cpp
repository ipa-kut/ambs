#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include "ambs_runners/test1_braking.h"

namespace ambs_runners
{

class Test1BrakingNodelet : public nodelet::Nodelet
{
public:
  Test1BrakingNodelet() {}
  ~Test1BrakingNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        class_pointer_.reset(new Test1Braking(nh, name));
        class_pointer_->init();
      }

private:
    boost::shared_ptr<Test1Braking> class_pointer_;
};

}  // namespace ambs_runners

PLUGINLIB_EXPORT_CLASS(ambs_runners::Test1BrakingNodelet,
                       nodelet::Nodelet);
