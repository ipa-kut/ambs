#ifndef EDGE_DETECTOR_H
#define EDGE_DETECTOR_H

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
  void init();

private:
  void executeCB(const ros::TimerEvent& event) override;
  ros::NodeHandle nh_;
  ambs_base::AMBSBooleanInterface bool_interface_;
  const std::string OUT_RISING_ = "out_rising";
  const std::string OUT_FALLING_ = "out_falling";
  const std::string IN_BOOL_ = "in_bool";
  const std::string PARAM_ = "param";
  const std::string TOLERANCE_ = "tolerance";
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
void EdgeDetector::init()
{
  std::vector<std::string> bool_inputs{IN_BOOL_};
  std::vector<std::string> bool_outputs{OUT_RISING_, OUT_FALLING_};
  bool_interface_.init(bool_inputs, bool_outputs, nh_, node_name_);

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

  ROS_INFO_STREAM(node_name_ << ": Restarting calculator");
}



}  // namespace ambs_calculators

#endif // EDGE_DETECTOR_H
