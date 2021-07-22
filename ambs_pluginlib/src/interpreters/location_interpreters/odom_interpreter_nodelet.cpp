#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include "ambs_components/ambs_interpreters/location_interpreters/odom_interpreter.h"

namespace ambs_interpreters
{

class OdomInterpreterNodelet : public nodelet::Nodelet
{
public:
  OdomInterpreterNodelet() {}
  ~OdomInterpreterNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        class_object.reset(new OdomInterpreter(nh, name));
        class_object->init();
      }

private:
    boost::shared_ptr<OdomInterpreter> class_object;
};

}  // namespace ambs_interpreters

PLUGINLIB_EXPORT_CLASS(ambs_interpreters::OdomInterpreterNodelet,
                       nodelet::Nodelet);
