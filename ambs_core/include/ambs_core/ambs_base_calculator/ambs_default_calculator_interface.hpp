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
  void initDefaultInterface(ros::NodeHandle nh, std::string node_name);

private:
  const std::string START_ = "/in_start"; ///< The START port
  const std::string STOP_ = "/in_stop"; ///< The STOP port
  const std::string RESET_ = "/in_reset"; ///< The RESET port
  const std::string DONE_ = "/out_done"; ///< The DONE port
  const std::vector<std::string> default_inputs_{START_, STOP_, RESET_};
  const std::vector<std::string> default_outputs_{DONE_};
};

// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------

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
  ROS_INFO_STREAM(node_name_ <<  ": Wait for Stop");
  ambs_msgs::BoolStamped msg = waitForTrueOnPort(STOP_);
  ROS_INFO_STREAM(node_name_ << ": Got Stop");
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
 * @brief Initialisation decoupled from constructor for convenince
 *
 */
inline void AMBSDefaultCalculatorInterface::initDefaultInterface(ros::NodeHandle nh, std::string node_name)
{
  init(default_inputs_,default_outputs_, nh, node_name);
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
