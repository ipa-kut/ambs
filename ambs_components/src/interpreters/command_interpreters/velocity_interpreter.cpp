#include <string>
#include <vector>

#include "ambs_components/ambs_interpreters/command_interpreters/velocity_interpreter.h"

namespace ambs_interpreters {

CommandVelocityInterpreter::CommandVelocityInterpreter(ros::NodeHandle nh, std::string node_name):
  nh_(nh),
  node_name_(node_name)
{}

void CommandVelocityInterpreter::init()
{
  ROS_INFO_STREAM(node_name_ << ": Interpreter ready");
  std::string param_name = "/ambs/interpreters/" + node_name_ + "/" + PARAM_VEL_X_;
  if (!nh_.param(param_name, velocity_x_, default_velocity_x_))
  {
    ROS_WARN_STREAM(node_name_ << ": Param " << param_name << " not found, defaulting to " << default_velocity_x_);
  }

  std::vector<std::string> inputs{START_ROBOT_, STOP_ROBOT_, ACTIVATE_, DEACTIVATE_};
  std::vector<std::string> outputs{};

  control_interface_.init(inputs, outputs, nh_, node_name_);
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>(CMD_VEL_, 10);
  execute_timer_ = nh_.createTimer(ros::Duration(1/pub_rate_),
                                   boost::bind(&CommandVelocityInterpreter::executeCB, this, _1));
}

void CommandVelocityInterpreter::executeCB(const ros::TimerEvent &event)
{
  (void) event;

  if (control_interface_.getPortMsg(ACTIVATE_).data &&
      !control_interface_.getPortMsg(DEACTIVATE_).data)
  {
    ROS_DEBUG_STREAM(node_name_ << ": ACTIVATED");
    if (control_interface_.getPortMsg(START_ROBOT_).data &&
        !control_interface_.getPortMsg(STOP_ROBOT_).data)
    {
      ROS_DEBUG_STREAM(node_name_ << ": Pub " << velocity_x_);
      pub_vel_.publish(constructNewTwistWithX(velocity_x_));
    }
    else if (!control_interface_.getPortMsg(START_ROBOT_).data &&
             control_interface_.getPortMsg(STOP_ROBOT_).data)
    {
      ROS_DEBUG_STREAM(node_name_ << ": Pub " << 0);
      pub_vel_.publish(constructNewTwistWithX(0));
    }
    else if (control_interface_.getPortMsg(START_ROBOT_).data &&
             control_interface_.getPortMsg(STOP_ROBOT_).data)
    {
      ROS_DEBUG_STREAM(node_name_ << ": Start & Stop both true!!");
    }
    else
    {
      ROS_DEBUG_STREAM(node_name_ << ": Pub nothing");
    }
    return;
  }
  else if (control_interface_.getPortMsg(ACTIVATE_).data &&
           control_interface_.getPortMsg(DEACTIVATE_).data)
  {
    ROS_INFO_STREAM(node_name_ << ": DEACTIVATED");
    control_interface_.resetAllPorts();
    return;
  }
}

geometry_msgs::Twist CommandVelocityInterpreter::constructNewTwistWithX(double x)
{
  geometry_msgs::Twist msg;
  msg.linear.x = x;
  return msg;
}

}  // namespace ambs_interpreters
