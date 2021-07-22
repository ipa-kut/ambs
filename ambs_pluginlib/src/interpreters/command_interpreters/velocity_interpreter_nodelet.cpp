#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include "ambs_interpreters/command_interpreters/velocity_interpreter.h"

namespace ambs_interpreters
{

class CommandVelocityInterpreterNodelet : public nodelet::Nodelet
{
public:
  CommandVelocityInterpreterNodelet() {}
  ~CommandVelocityInterpreterNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        class_object.reset(new CommandVelocityInterpreter(nh, name));
        class_object->init();
      }

private:
    boost::shared_ptr<CommandVelocityInterpreter> class_object;
};

}  // namespace ambs_interpreters

PLUGINLIB_EXPORT_CLASS(ambs_interpreters::CommandVelocityInterpreterNodelet,
                       nodelet::Nodelet);
