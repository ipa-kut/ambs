#ifndef AMBS_BOOL_INTERFACE_HPP
#define AMBS_BOOL_INTERFACE_HPP

#include "ambs_core/ambs_base_interface/ambs_boolean_interface.hpp"
#include "ambs_msgs/BoolStamped.h"
#include <std_msgs/Header.h>

namespace ambs_base
{
/**
 * @brief A standard boolean AMBSTemplatedInterface interface for calculators
 * @todo Merge this into AMBSBaseCalculator
 *
 * Useful because all components should have a standard boolean control interface such as start, stop, reset & done
 */
class AMBSDefaultCalculatorInterface : public AMBSBooleanInterface
{
public:
  AMBSDefaultCalculatorInterface() {}
  virtual ~AMBSDefaultCalculatorInterface() {}

  ambs_msgs::BoolStamped getStopMsg();
  ambs_msgs::BoolStamped getStartMsg();
  ambs_msgs::BoolStamped waitForStart();
  ambs_msgs::BoolStamped waitForStop();
  ambs_msgs::BoolStamped waitForReset();
  void publishDone();
  void printDefaultPorts();
  void initDefaultInterface(ros::NodeHandle nh,
                            std::string node_name,
                            std::string in_start,
                            std::string in_stop,
                            std::string in_reset,
                            std::string out_done);

private:
  std::string START_;
  std::string STOP_;
  std::string RESET_;
  std::string DONE_;

};

// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Initialisation decoupled from constructor for convenince
 *
 */
inline void AMBSDefaultCalculatorInterface::initDefaultInterface(ros::NodeHandle nh,
                                                                 std::string node_name,
                                                                 std::string in_start = "in_start",
                                                                 std::string in_stop = "in_stop",
                                                                 std::string in_reset = "in_reset",
                                                                 std::string out_done = "out_done")
{
  START_ = in_start;
  STOP_ = in_stop;
  RESET_ = in_reset;
  DONE_ = out_done;
  std::vector<std::string> default_inputs{START_, STOP_, RESET_};
  std::vector<std::string> default_outputs{DONE_};
  init(default_inputs, default_outputs, nh, node_name);
}

/**
 * @brief Wait for TRUE on the START port
 * @return The msg that set TRUE
 */
inline ambs_msgs::BoolStamped AMBSDefaultCalculatorInterface::waitForStart()
{
  ROS_INFO_STREAM(node_name_ << ": Wait for Start");
  ambs_msgs::BoolStamped msg = waitForTrueOnPort(START_);
  ROS_INFO_STREAM(node_name_ << ": Got Start");
  return msg;
}

/**
 * @brief Wait for TRUE on the STOP port
 * @return The msg that set TRUE
 */
inline ambs_msgs::BoolStamped AMBSDefaultCalculatorInterface::waitForStop()
{
  ambs_msgs::BoolStamped msg = waitForTrueOnPort(STOP_);
  return msg;
}

/**
 * @brief Wait for TRUE on the RESET port
 * @return The msg that set TRUE
 */
inline ambs_msgs::BoolStamped AMBSDefaultCalculatorInterface::waitForReset()
{
  ROS_INFO_STREAM(node_name_ <<  ": Wait for Reset");
  ambs_msgs::BoolStamped msg = waitForTrueOnPort(RESET_);
  ROS_INFO_STREAM(node_name_ << ": Got Reset");
  resetAllPorts();
  return msg;
}

/**
 * @brief Publish TRUE on the DONE port
 *
 */
inline void AMBSDefaultCalculatorInterface::publishDone()
{
  publishMsgOnPort(DONE_, constructNewBoolStamped(true));
}

/**
 * @brief Print internal buffer values of all ports
 *
 * Used mainly for debugging
 */
inline void AMBSDefaultCalculatorInterface::printDefaultPorts()
{
  ROS_INFO_STREAM("Start : " << std::to_string(getPortMsg(START_).data)
                  << " Stop : " << std::to_string(getPortMsg(STOP_).data)
                  << " Reset : " << std::to_string(getPortMsg(RESET_).data)
                  << " Done : " << std::to_string(getPortMsg(DONE_).data));
}

/**
 * @brief Get the message received by the STOP_ port
 * @returns msg The message
 */
inline ambs_msgs::BoolStamped AMBSDefaultCalculatorInterface::getStopMsg()
{
  return getPortMsg(STOP_);
}

/**
 * @brief Get the message received by the START_ port
 * @returns msg The message
 */
inline ambs_msgs::BoolStamped AMBSDefaultCalculatorInterface::getStartMsg()
{
  return getPortMsg(START_);
}


}  // namespace ambs_base

#endif // AMBS_BOOL_INTERFACE_HPP
