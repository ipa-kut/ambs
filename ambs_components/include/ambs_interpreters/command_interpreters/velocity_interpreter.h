#ifndef AMBS_INTERPRETERS_COMMAND_INTERPRETERS_VELOCITY_INTERPRETER_H
#define AMBS_INTERPRETERS_COMMAND_INTERPRETERS_VELOCITY_INTERPRETER_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "ambs_msgs/BoolStamped.h"
#include "ambs_core/ambs_base_interface/ambs_boolean_interface.hpp"


namespace ambs_interpreters {

/**
 * @brief Class to convert velocity commands from runner into cmd_vel messages
 *
 * Ports:
 * Inputs - Activate, Start, Stop, Deactivate
 * Outputs - Cmd_vel
 *
 * Functionality:
 * All input ports have an internal buffer that saves latest value.
 * Interpreter always works off of latest buffer value.
 * Start & Stop are matched pair, needing to be only one True and one False to work.
 * Starts deactivated state.
 *
 * Operation:
 * Set Activate=True to enable interpreter
 * If Activated, and if neither Start nor Stop is True, send no velocity
 * If Activated, set (Start=True then Stop=False) to start sending parametrised velocity
 * If Activated, set (Start=False then Stop=True) to start sending 0 velocity
 * If Activated, set Deactivate=True to disable interpreter, send no velocity, clear all buffers &
 *      go back to deactivated state.
 */
class CommandVelocityInterpreter
{
public:
  CommandVelocityInterpreter() {}
  CommandVelocityInterpreter(ros::NodeHandle nh, std::string node_name);

  void init();

private:
  ros::NodeHandle nh_;
  ros::Timer execute_timer_;
  ros::Publisher pub_vel_;
  std::string node_name_;
  ambs_base::AMBSBooleanInterface control_interface_;
  double velocity_x_;

  const std::string START_ROBOT_ = "in_start_robot";
  const std::string STOP_ROBOT_ = "in_stop_robot";
  const std::string ACTIVATE_ = "in_activate";
  const std::string DEACTIVATE_ = "in_deactivate";
  const std::string CMD_VEL_ = "out_cmd_vel";

  const std::string PARAM_VEL_X_ = "velocity_x";
  const double default_velocity_x_ = 0.8;
  const double queue_size = 10;
  const double pub_rate_ = 20;

  void executeCB(const ros::TimerEvent& event);
  geometry_msgs::Twist constructNewTwistWithX(double x);
};

}  // namespace ambs_interpreters

#endif  // AMBS_INTERPRETERS_COMMAND_INTERPRETERS_VELOCITY_INTERPRETER_H
