#ifndef AMBSBASEBOOLEANINTERFACE_HPP
#define AMBSBASEBOOLEANINTERFACE_HPP

#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "ambs_msgs/BoolStamped.h"

namespace ambs_base {

/**
 * @brief A bool specialisation of AMBSTemplatedInterface. Offers two extra functions
 */
class AMBSBooleanInterface : public AMBSTemplatedInterface<ambs_msgs::BoolStamped>
{
public:
  AMBSBooleanInterface() {}
  AMBSBooleanInterface(ros::NodeHandle nh,
                       std::string node_name,
                       std::vector<std::string> extended_bool_inputs,
                       std::vector<std::string> extended_bool_outputs);
  virtual ~AMBSBooleanInterface() {}

  ambs_msgs::BoolStamped constructNewBoolStamped(bool data);
  ambs_msgs::BoolStamped waitForTrueOnPort(std::string port);

private:
  const double wait_loop_rate = 500;
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


inline AMBSBooleanInterface::AMBSBooleanInterface(ros::NodeHandle nh,
                                           std::string node_name,
                                           std::vector<std::string> extended_bool_inputs,
                                           std::vector<std::string> extended_bool_outputs)
{
  init(extended_bool_inputs, extended_bool_outputs, nh, node_name);
}

/**
 * @brief Create a new BoolStamped type message
 *
 * @param[in] data The data to be held by the new msg
 * @returns msg The message to be returned
 */
inline ambs_msgs::BoolStamped AMBSBooleanInterface::constructNewBoolStamped(bool data)
{
  ambs_msgs::BoolStamped msg;
  msg.data  = data;
  msg.header = std_msgs::Header();
  msg.header.stamp = ros::Time::now();
  return msg;
}

/**
 * @brief Wait for TRUE on a port
 * @param port The port to wait on
 * @returns The msg that set TRUE
 */
inline ambs_msgs::BoolStamped AMBSBooleanInterface::waitForTrueOnPort(std::string port)
{
  std::unique_lock<std::mutex> lock(mutexes_[interfaces_[port].index_]);
  cond_vars_[interfaces_[port].index_].wait(lock, [this, port]() -> bool{return getPortMsg(port).data;});
  ambs_msgs::BoolStamped msg = *interfaces_[port].msg_;
  resetPort(port);
  return msg;
}

}  // namespace ambs_base
#endif // AMBSBASEBOOLEANINTERFACE_HPP
