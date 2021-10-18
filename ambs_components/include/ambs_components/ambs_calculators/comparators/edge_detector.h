#ifndef AMBS_COMPONENTS_AMBS_CALCULATORS_COMPARATORS_EDGE_DETECTOR_H
#define AMBS_COMPONENTS_AMBS_CALCULATORS_COMPARATORS_EDGE_DETECTOR_H

#include <string>
#include <vector>

#include "ambs_core/ambs_base_calculator/ambs_base_calculator.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "ambs_msgs/BoolStamped.h"

namespace ambs_calculators
{

/**
 * @brief Monitor a boolean input and output detected rising/falling edges. Cannot detec first falling edge.
 *
 * Has standard control intetrface, and a bool interface with one input and two output ports
 */
class EdgeDetector : public ambs_base::AMBSBaseCalculator
{
public:
  EdgeDetector() {}
  ~EdgeDetector() {}
  EdgeDetector(ros::NodeHandle nh, std::string node_name):
    ambs_base::AMBSBaseCalculator(nh, node_name),
    nh_(nh)
  {}
  void init(std::string in_start,
            std::string in_stop,
            std::string in_reset,
            std::string out_done,
            std::string in_bool,
            std::string out_rising,
            std::string out_falling);

private:
  void executeCB(const ros::TimerEvent& event) override;
  ros::NodeHandle nh_;
  ambs_base::AMBSBooleanInterface bool_interface_;
  std::string OUT_RISING_ = "out_rising";
  std::string OUT_FALLING_ = "out_falling";
  std::string IN_BOOL_ = "in_bool";
  bool previous_bool_ = false;
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


/**
 * @brief init function will be called by nodelet, so it is not called anywhere from within class
 *
 * AMBSBaseCalculator::startCalculator() spawns a timer which executes executeCB()
 */
void EdgeDetector::init(std::string in_start = "in_start",
                        std::string in_stop = "in_stop",
                        std::string in_reset = "in_reset",
                        std::string out_done = "out_done",
                        std::string in_bool = "in_bool",
                        std::string out_rising = "out_rising",
                        std::string out_falling = "out_falling")
{
  OUT_RISING_ = out_rising;
  OUT_FALLING_ = out_falling;
  IN_BOOL_ = in_bool;

  std::vector<std::string> bool_inputs{IN_BOOL_};
  std::vector<std::string> bool_outputs{OUT_RISING_, OUT_FALLING_};
  bool_interface_.init(bool_inputs, bool_outputs, nh_, node_name_);
  default_control_.initDefaultInterface(nh_, node_name_, in_start, in_stop, in_reset, out_done);

  startCalculator();
}

/**
 * @brief Main calculator logic, called by the timer. Overrides AMBSBaseCalculator::executeCB()
 * @param event Not used
 */
void EdgeDetector::executeCB(const ros::TimerEvent& event)
{
  (void) event;
  default_control_.waitForStart();


  ros::Rate loop(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop.sleep();

     if (default_control_.getStopMsg().data)
     {
       ROS_INFO_STREAM(node_name_ << ": Got stop");
       break;
     }

     if (!bool_interface_.isPortValid(IN_BOOL_))
     {
       continue;
     }

     bool current_bool = bool_interface_.getPortMsg(IN_BOOL_).data;

     if (current_bool != previous_bool_)
     {
       if (current_bool)
       {
         ROS_INFO_STREAM(node_name_ << ": Rising edge detected");
         bool_interface_.publishMsgOnPort(OUT_FALLING_, default_control_.constructNewBoolStamped(false));
         bool_interface_.publishMsgOnPort(OUT_RISING_, default_control_.constructNewBoolStamped(true));
       }
       else
       {
         ROS_INFO_STREAM(node_name_ << ": Falling edge detected");
         bool_interface_.publishMsgOnPort(OUT_RISING_, default_control_.constructNewBoolStamped(false));
         bool_interface_.publishMsgOnPort(OUT_FALLING_, default_control_.constructNewBoolStamped(true));
       }
       previous_bool_ = current_bool;
     }
  }

  default_control_.publishDone();
  default_control_.waitForReset();
  bool_interface_.resetAllPorts();

  ROS_INFO_STREAM(node_name_ << ": Restarting calculator");
}



}  // namespace ambs_calculators

#endif  // AMBS_COMPONENTS_AMBS_CALCULATORS_COMPARATORS_EDGE_DETECTOR_H
