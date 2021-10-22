#ifndef AMBS_COMPONENTS_AMBS_CALCULATORS_UTILS_OR_GATE_H
#define AMBS_COMPONENTS_AMBS_CALCULATORS_UTILS_OR_GATE_H

#include <string>
#include <vector>

#include "ambs_core/ambs_base_calculator/ambs_base_calculator.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "ambs_msgs/BoolStamped.h"

namespace ambs_calculators
{

/**
 * @brief N Input OR Gate - 'nuff said.
 *
 * Takes param 'n_inputs' once during init and dynamically spawns as many inputs in runtime.
 */
class OrGate : public ambs_base::AMBSBaseCalculator
{
public:
  OrGate() {}
  ~OrGate() {}
  OrGate(ros::NodeHandle nh, std::string node_name):
    ambs_base::AMBSBaseCalculator(nh, node_name),
    nh_(nh)
  {}
  void init(std::string in_start,
            std::string in_stop,
            std::string in_reset,
            std::string out_done,
            std::string out_signal);

private:
  void executeCB(const ros::TimerEvent& event) override;
  ros::NodeHandle nh_;
  ambs_base::AMBSBooleanInterface bool_interface_;
  std::string OUT_RESULT_ = "out_signal";
  std::string PARAM_N_INPUTS_ = "n_inputs";
  int n_inputs_ = 3;  // Default 3 inputs
  std::vector<std::string> bool_inputs_;
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


/**
 * @brief init function will be called by nodelet, so it is not called anywhere from within class
 *
 * AMBSBaseCalculator::startCalculator() spawns a timer which executes executeCB()
 */
void OrGate::init(std::string in_start = "in_start",
                         std::string in_stop = "in_stop",
                         std::string in_reset = "in_reset",
                         std::string out_done = "out_done",
                         std::string out_signal = "out_signal")
{
  OUT_RESULT_ = out_signal;

  n_inputs_ = getResolvedParam<int>(PARAM_N_INPUTS_, n_inputs_);
  if (n_inputs_ < 2)
  {
    ROS_ERROR_STREAM(node_name_ + ": Param " << PARAM_N_INPUTS_ << "<2, makes no sense");
    throw std::invalid_argument(node_name_ + ": Param " + PARAM_N_INPUTS_ + "<2, makes no sense");
  }

  for (int i = 1; i <= n_inputs_; i++)
  {
    bool_inputs_.push_back("in_input_"+std::to_string(i));
  }

  std::vector<std::string> bool_outputs{OUT_RESULT_};
  bool_interface_.init(bool_inputs_, bool_outputs, nh_, node_name_);
  default_control_.initDefaultInterface(nh_, node_name_, in_start, in_stop, in_reset, out_done);

  startCalculator();
}

/**
 * @brief Main calculator logic, called by the timer. Overrides AMBSBaseCalculator::executeCB()
 * @param event Not used
 */
void OrGate::executeCB(const ros::TimerEvent& event)
{
  (void) event;
  default_control_.waitForStart();

  ros::Rate loop(100);
  while (ros::ok())
  {
    loop.sleep();

    if (default_control_.getStopMsg().data)
    {
      ROS_INFO_STREAM(node_name_ << ": Got stop");
      break;
    }

    for (std::string port : bool_inputs_)
    {
      if (bool_interface_.getPortMsg(port).data)
      {
        bool_interface_.publishMsgOnPort(OUT_RESULT_, bool_interface_.constructNewBoolStamped(true));
        bool_interface_.resetAllPorts();
      }
    }
  }

  bool_interface_.resetAllPorts();
  default_control_.publishDone();
  default_control_.waitForReset();

  ROS_INFO_STREAM(node_name_ << ": Restarting calculator");
}

}  // namespace ambs_calculators

#endif  // AMBS_COMPONENTS_AMBS_CALCULATORS_UTILS_OR_GATE_H
