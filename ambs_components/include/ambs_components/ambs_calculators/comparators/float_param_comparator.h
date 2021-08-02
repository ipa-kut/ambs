#ifndef AMBS_COMPONENTS_AMBS_CALCULATORS_COMPARATORS_FLOAT_PARAM_COMPARATOR_H
#define AMBS_COMPONENTS_AMBS_CALCULATORS_COMPARATORS_FLOAT_PARAM_COMPARATOR_H

#include <string>
#include <vector>

#include "ambs_core/ambs_base_calculator/ambs_base_calculator.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "std_msgs/Float64.h"

namespace ambs_calculators
{

/**
 * @brief Compare float from topic vs a param and output true when nearly equal
 *
 * Has standard control intetrface, and a float interface with one input and one output port,
 * and two rosparams
 */
class CompFloatParam : public ambs_base::AMBSBaseCalculator
{
public:
  CompFloatParam() {}
  ~CompFloatParam() {}
  CompFloatParam(ros::NodeHandle nh, std::string node_name):
    ambs_base::AMBSBaseCalculator(nh, node_name),
    nh_(nh)
  {}
  void init();

private:
  void executeCB(const ros::TimerEvent& event) override;
  ros::NodeHandle nh_;
  ambs_base::AMBSBooleanInterface bool_interface_;
  ambs_base::AMBSTemplatedInterface<std_msgs::Float64> float_interface_;
  std_msgs::Float64 result_msg_;
  const std::string COMPARISON_ = "out_comparison";
  const std::string IN_FLOAT_ = "in_float";  ///< Input float topic
  const std::string PARAM_ = "param";
  const std::string TOLERANCE_ = "tolerance";
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


/**
 * @brief init function will be called by nodelet, so it is not called anywhere from within class
 *
 * AMBSBaseCalculator::startCalculator() spawns a timer which executes executeCB()
 */
void CompFloatParam::init()
{
  std::vector<std::string> float_inputs{IN_FLOAT_};
  std::vector<std::string> float_outputs;
  float_interface_.init(float_inputs, float_outputs, nh_, node_name_);

  std::vector<std::string> bool_inputs;
  std::vector<std::string> bool_outputs{COMPARISON_};
  bool_interface_.init(bool_inputs, bool_outputs, nh_, node_name_);

  startCalculator();
}

/**
 * @brief Main calculator logic, called by the timer. Overrides AMBSBaseCalculator::executeCB()
 * @param event Not used
 */
void CompFloatParam::executeCB(const ros::TimerEvent& event)
{
  (void) event;
  default_control_.waitForStart();

  float param = getResolvedParam<float>(PARAM_);
  float tolerance = getResolvedParam<float>(TOLERANCE_);
  bool is_published = false;

  ros::Rate loop(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop.sleep();

     if (default_control_.getStopMsg().data)
     {
       ROS_INFO_STREAM(node_name_ << ": Got stop");
       break;
     }

     if (!float_interface_.isPortValid(IN_FLOAT_))
     {
       continue;
     }

     float float_val = float_interface_.getPortMsg(IN_FLOAT_).data;
     if (std::abs(float_val - param) <= tolerance)
     {
       if (!is_published)
       {
         ROS_INFO_STREAM(node_name_ << ": Comparison TRUE");
         bool_interface_.publishMsgOnPort(COMPARISON_, default_control_.constructNewBoolStamped(true));
         is_published = true;
       }
     }
     else if (is_published)
     {
       ROS_INFO_STREAM(node_name_ << ": Comparison FALSE");
       bool_interface_.publishMsgOnPort(COMPARISON_, default_control_.constructNewBoolStamped(false));
       is_published = false;
     }
  }

  default_control_.publishDone();
  default_control_.waitForReset();
  float_interface_.resetAllPorts();

  ROS_INFO_STREAM(node_name_ << ": Restarting calculator");
}



}  // namespace ambs_calculators


#endif  // AMBS_COMPONENTS_AMBS_CALCULATORS_COMPARATORS_FLOAT_PARAM_COMPARATOR_H
