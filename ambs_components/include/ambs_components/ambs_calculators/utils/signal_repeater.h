#ifndef AMBS_COMPONENTS_AMBS_CALCULATORS_UTILS_SIGNAL_REPEATER_H
#define AMBS_COMPONENTS_AMBS_CALCULATORS_UTILS_SIGNAL_REPEATER_H

#include <string>
#include <vector>

#include "ambs_core/ambs_base_calculator/ambs_base_calculator.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "ambs_msgs/BoolStamped.h"

namespace ambs_calculators
{

/**
 * @brief Boost a signal to a certain frequency
 *
 * @todo Make this work with any type of topic (Possible?). Check if boosting up or down.
 */
class SignalRepeater : public ambs_base::AMBSBaseCalculator
{
public:
  SignalRepeater() {}
  ~SignalRepeater() {}
  SignalRepeater(ros::NodeHandle nh, std::string node_name):
    ambs_base::AMBSBaseCalculator(nh, node_name),
    nh_(nh)
  {}
  void init(std::string in_start,
            std::string in_stop,
            std::string in_reset,
            std::string out_done,
            std::string in_signal,
            std::string out_signal);

private:
  void executeCB(const ros::TimerEvent& event) override;
  ros::NodeHandle nh_;
  ambs_base::AMBSBooleanInterface bool_interface_;
  std::string OUT_SIGNAL_ = "out_signal";
  std::string IN_SIGNAL_ = "in_signal";
  const std::string RATE_ = "boost_rate";
  const int DEFAULT_RATE_ = 20;
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


/**
 * @brief init function will be called by nodelet, so it is not called anywhere from within class
 *
 * AMBSBaseCalculator::startCalculator() spawns a timer which executes executeCB()
 */
void SignalRepeater::init(std::string in_start = "in_start",
                         std::string in_stop = "in_stop",
                         std::string in_reset = "in_reset",
                         std::string out_done = "out_done",
                         std::string in_signal = "in_signal",
                         std::string out_signal = "out_signal")
{
  OUT_SIGNAL_ = out_signal;
  IN_SIGNAL_ = in_signal;

  std::vector<std::string> bool_inputs{IN_SIGNAL_};
  std::vector<std::string> bool_outputs{OUT_SIGNAL_};
  bool_interface_.init(bool_inputs, bool_outputs, nh_, node_name_);
  default_control_.initDefaultInterface(nh_, node_name_, in_start, in_stop, in_reset, out_done);

  startCalculator();
}

/**
 * @brief Main calculator logic, called by the timer. Overrides AMBSBaseCalculator::executeCB()
 * @param event Not used
 */
void SignalRepeater::executeCB(const ros::TimerEvent& event)
{
  (void) event;
  default_control_.waitForStart();

  ros::Rate loop(getResolvedParam<int>(RATE_, DEFAULT_RATE_));
  while (ros::ok())
  {
    loop.sleep();

    if (default_control_.getStopMsg().data)
    {
      ROS_INFO_STREAM(node_name_ << ": Got stop");
      break;
    }

    bool val_to_pub = bool_interface_.isPortValid(IN_SIGNAL_) ? bool_interface_.getPortMsg(IN_SIGNAL_).data : false;
    bool_interface_.publishMsgOnPort(OUT_SIGNAL_, bool_interface_.constructNewBoolStamped(val_to_pub));
  }

  default_control_.publishDone();
  default_control_.waitForReset();

  ROS_INFO_STREAM(node_name_ << ": Restarting calculator");
}



}  // namespace ambs_calculators

#endif  // AMBS_COMPONENTS_AMBS_CALCULATORS_UTILS_SIGNAL_REPEATER_H
