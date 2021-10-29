#ifndef AMBS_BASE_CALCULATOR_HPP
#define AMBS_BASE_CALCULATOR_HPP

#include <thread>
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "ambs_core/ambs_base_calculator/ambs_default_calculator_interface.hpp"
#include "ambs_msgs/BoolStamped.h"
#include <std_msgs/Header.h>

namespace ambs_base
{
/**
 * @brief The VIRTUAL base class for all calculators to inherit from
 * @todo Add parameter container + setters/getters
 *
 * Creates a standard_control_ interface of type AMBSStandardControlInterface
 * and a threaded timer to execute calculator logic
 */
class AMBSBaseCalculator
{
public:
  AMBSBaseCalculator() {}
  virtual ~AMBSBaseCalculator() {}
  AMBSBaseCalculator(ros::NodeHandle nh, std::string node_name);

protected:
  virtual void executeCB() = 0;
  void threadCB();
  void startCalculator();
  std::string node_name_;
  ros::NodeHandle nh_;
  template<typename T>
  T getResolvedParam(std::string param_name, T default_value = T());
  AMBSDefaultCalculatorInterface default_control_;

private:
  std::thread calc_thread_;
  const int thread_loop_rate_ = 100;
};

// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Constructor
 *
 * startCalculator() MUST BE EXPLICITLY called to start the timer
 *
 * @param nh NodeHandle received from the nodelet manager
 * @param node_name Explicitly necessary becase all nodelets resolve ros::this_node::getName() as nodelet manager
 */
inline  AMBSBaseCalculator::AMBSBaseCalculator(ros::NodeHandle nh, std::string node_name):
  node_name_(node_name),
  nh_(nh)
{
  default_control_.initDefaultInterface(nh_, node_name_);
}

/**
 * @brief Thread function. Runs calculator logic at @param thread_loop_rate Hz.
 */
inline void AMBSBaseCalculator::threadCB()
{
  while (ros::ok())
  {
    ros::Rate(thread_loop_rate_).sleep();
    executeCB();
  }

}

/**
 * @brief Starts the thread. MUST BE EXPLICITLY CALLED.
 */
inline void AMBSBaseCalculator::startCalculator()
{
  calc_thread_ = std::thread(&AMBSBaseCalculator::threadCB, this);
}

/**
 * @brief Gets a specified parameter by resolving to the ambs/calculators namespace
 *
 * @param param_name The parameter to retrieve
 * @tparam T nh.getParam() supports ONLY STRING, INT, DOUBLE, FLOAT and BOOL or vectors thereof
 *
 * example: getResolvedParam<std::vector<double>>("some_param")
 */
template<typename T> inline
T AMBSBaseCalculator::getResolvedParam(std::string param_name, T default_value)
{
  std::string resolved_name = "/ambs/calculators/" + node_name_ + "/" + param_name;
  if(nh_.hasParam(resolved_name))
  {
    T param_value;
    nh_.getParam(resolved_name, param_value);
    return param_value;
  }
  else
  {
    ROS_WARN_STREAM(node_name_ << ": Could not fetch param "
                    << resolved_name << ", using default " << default_value);
    return default_value;
  }
}


} // namespace ambs_base

#endif // AMBS_BASE_CALCULATOR_HPP
