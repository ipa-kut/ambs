#ifndef AMBS_COMPONENTS_AMBS_CALCULATORS_DIFFERENTIATORS_STOPWATCH_H
#define AMBS_COMPONENTS_AMBS_CALCULATORS_DIFFERENTIATORS_STOPWATCH_H

#include <string>
#include <vector>
#include <std_msgs/Float64.h>

#include "ambs_core/ambs_base_calculator/ambs_base_calculator.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "ambs_components/ambs_loggers/debug_logger.h"

namespace ambs_calculators
{

/**
 * @brief Compute difference in Time between start and stop, taken from messages directly
 *
 * Has standard control intetrface, and a float interface with one output port
 */
class Stopwatch : public ambs_base::AMBSBaseCalculator
{
public:
  Stopwatch() {}
  ~Stopwatch() {}
  Stopwatch(ros::NodeHandle nh, std::string node_name):
    ambs_base::AMBSBaseCalculator(nh, node_name),
    nh_(nh)
  {}
  void init();

private:
  void executeCB();
  ros::NodeHandle nh_;
  ambs_base::AMBSTemplatedInterface<std_msgs::Float64> float_interface_;
  std_msgs::Float64 result_msg_;
  const std::string OUT_FLOAT_ = "out_float";  ///< Output float topic
  ambs_loggers::DebugLogger debug_logger_;
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


/**
 * @brief init function will be called by nodelet manager, so it is not called anywhere from within class
 *
 * AMBSBaseCalculator::startCalculator() spawns a timer which executes executeCB()
 */
void Stopwatch::init()
{
  std::vector<std::string> float_inputs{};
  std::vector<std::string> float_outputs{OUT_FLOAT_};
  float_interface_.init(float_inputs, float_outputs, nh_, node_name_);
  debug_logger_.init(nh_, node_name_);

  startCalculator();
}

/**
 * @brief Main calculator logic, called by the timer. Overrides AMBSBaseCalculator::executeCB()
 * @param event Not used
 */
void Stopwatch::executeCB()
{
  ros::Time start_time = default_control_.waitForStart().header.stamp;
  ROS_DEBUG_STREAM(node_name_ << ": Got start time: " << start_time);

  ros::Time stop_time = default_control_.waitForStop().header.stamp;
  ROS_DEBUG_STREAM(node_name_ << ": Got stop time: " << stop_time);

  ros::Duration difference = stop_time - start_time;
  ROS_INFO_STREAM(node_name_ << ": Difference: " << difference);

  result_msg_.data = difference.toSec();
  float_interface_.publishMsgOnPort(OUT_FLOAT_, result_msg_);

  debug_logger_.logInfo(std::to_string(result_msg_.data));

  default_control_.publishDone();
  default_control_.waitForReset();
  float_interface_.resetAllPorts();

  ROS_INFO_STREAM(node_name_ << ": Restarting calculator");
}


}  // namespace ambs_calculators


#endif  // AMBS_COMPONENTS_AMBS_CALCULATORS_DIFFERENTIATORS_STOPWATCH_H
