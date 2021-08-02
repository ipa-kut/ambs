#ifndef AMBS_COMPONENTS_AMBS_CALCULATORS_DIFFERENTIATORS_DIFF_FLOAT_TEMPORAL_H
#define AMBS_COMPONENTS_AMBS_CALCULATORS_DIFFERENTIATORS_DIFF_FLOAT_TEMPORAL_H

#include <string>
#include <vector>

#include "ambs_core/ambs_base_calculator/ambs_base_calculator.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "std_msgs/Float64.h"

namespace ambs_calculators
{

/**
 * @brief Compute difference in a float value from a topic between start and stop
 *
 * Has standard control intetrface, and a float interface with one input and one output port
 */
class DiffFloatTemporal : public ambs_base::AMBSBaseCalculator
{
public:
  DiffFloatTemporal() {}
  ~DiffFloatTemporal() {}
  DiffFloatTemporal(ros::NodeHandle nh, std::string node_name):
    ambs_base::AMBSBaseCalculator(nh, node_name),
    nh_(nh)
  {}
  void init();

private:
  void executeCB(const ros::TimerEvent& event) override;
  ros::NodeHandle nh_;
  ambs_base::AMBSTemplatedInterface<std_msgs::Float64> float_interface_;
  std_msgs::Float64 result_msg_;
  const std::string IN_FLOAT_ = "in_float";  ///< Input float topic
  const std::string OUT_FLOAT_ = "out_float";  ///< Output float topic
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


/**
 * @brief init function will be called by nodelet, so it is not called anywhere from within class
 *
 * AMBSBaseCalculator::startCalculator() spawns a timer which executes executeCB()
 */
void DiffFloatTemporal::init()
{
  ROS_INFO_STREAM(node_name_ <<": Init class");

  std::vector<std::string> float_inputs{IN_FLOAT_};
  std::vector<std::string> float_outputs{OUT_FLOAT_};
  float_interface_.init(float_inputs, float_outputs, nh_, node_name_);

  startCalculator();
}

/**
 * @brief Main calculator logic, called by the timer. Overrides AMBSBaseCalculator::executeCB()
 * @param event Not used
 */
void DiffFloatTemporal::executeCB(const ros::TimerEvent& event)
{
  (void) event;
  default_control_.waitForStart();
  double start_float = float_interface_.getPortMsg(IN_FLOAT_).data;
  ROS_INFO_STREAM(node_name_ << ": Starting float stored: " << start_float);
  ROS_INFO(" ");

  default_control_.waitForStop();
  double stop_float = float_interface_.getPortMsg(IN_FLOAT_).data;
  ROS_INFO_STREAM(node_name_ << ": Stopping float stored: " << stop_float);
  ROS_INFO(" ");

  double diff_float = stop_float - start_float;
  ROS_INFO_STREAM(node_name_ << ": Difference in floats: " << diff_float);
  ROS_INFO(" ");
  result_msg_.data = diff_float;
  float_interface_.publishMsgOnPort(OUT_FLOAT_, result_msg_);

  default_control_.publishDone();
  default_control_.waitForReset();
  float_interface_.resetAllPorts();

  ROS_INFO_STREAM(node_name_ << ": Restarting calculator");
  ROS_INFO(" ");
  ROS_INFO("-----------------------------------------------------------");
  ROS_INFO(" ");
}


}  // namespace ambs_calculators

#endif  // AMBS_COMPONENTS_AMBS_CALCULATORS_DIFFERENTIATORS_DIFF_FLOAT_TEMPORAL_H
