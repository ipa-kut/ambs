#ifndef AMBS_COMPONENTS_AMBS_CALCULATORS_DIFFERENTIATORS_TIMER_H
#define AMBS_COMPONENTS_AMBS_CALCULATORS_DIFFERENTIATORS_TIMER_H


#include <string>
#include <vector>
#include <std_msgs/Float64.h>

#include "ambs_core/ambs_base_calculator/ambs_base_calculator.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "ambs_components/ambs_loggers/debug_logger.h"

namespace ambs_calculators
{
enum class TimerState
{
  DISABLED,
  IDLE,
  RUNNING,
  FINISHED,
};
/**
 * @brief Start counting down timer from a parametrized value to 0
 * when triggered and set timed out unless interrupted.
 *
 * Has standard control intetrface, a float interface with one output port and
 * a Bool interface with two inputs and two outputs.
 */
class Timer : public ambs_base::AMBSBaseCalculator
{
public:
  Timer() {}
  ~Timer() {}
  Timer(ros::NodeHandle nh, std::string node_name):
    ambs_base::AMBSBaseCalculator(nh, node_name),
    nh_(nh)
  {}
  void init();

private:
  void executeCB();
  void disableTimer();
  TimerState getNextState(TimerState current_state);

  ros::NodeHandle nh_;
  ambs_base::AMBSBooleanInterface control_interface_;
  ambs_base::AMBSTemplatedInterface<std_msgs::Float64> float_interface_;
  ambs_loggers::DebugLogger debug_logger_;

  const std::string ENABLE_ = "in_enable";
  const std::string DISABLE_ = "in_disable";
  const std::string TIMED_OUT_ = "out_timed_out";
  const std::string INTERRUPTED_ = "out_interrupted";
  const std::string ELAPSED_ = "out_elapsed";
  const std::string PARAM_DURATION_ = "duration";
  const std::string PARAM_DEBUG_ = "debug_states";
  bool debug_states_;
  const double default_duration_ = 5;

  ros::Time start_time_, current_time_;
  ros::Duration elapsed_;
  TimerState current_state_, next_state_;
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


/**
 * @brief init function will be called by nodelet manager, so it is not called anywhere from within class
 *
 * AMBSBaseCalculator::startCalculator() spawns a timer which executes executeCB()
 */
void Timer::init()
{
  std::vector<std::string> control_inputs{ENABLE_, DISABLE_};
  std::vector<std::string> control_outputs{TIMED_OUT_, INTERRUPTED_};
  control_interface_.init(control_inputs, control_outputs, nh_, node_name_);

  std::vector<std::string> float_inputs{};
  std::vector<std::string> float_outputs{ELAPSED_};
  float_interface_.init(float_inputs, float_outputs, nh_, node_name_);

  debug_logger_.init(nh_, node_name_);
  current_state_ = TimerState::DISABLED;  /// Always start from disabled state
  debug_states_ = getResolvedParam<bool>(PARAM_DEBUG_, false);

  startCalculator();
  ROS_INFO_STREAM(node_name_ << ": Waiting for Enable");
  if (debug_states_) debug_logger_.logInfo("DISABLED");
}

/**
 * @brief Calculate what the next state should be, given current state and external inputs
 *
 * The state machine is as follows, always starting from DISABLED:
 * DISABLED -> IDLE IFF ENABLE=True
 * IDLE -> RUNNING IFF START=True
 * RUNNING -> FINISHED IF STOP=True OR (elapsed >= duration)
 * <ANY STATE> -> DISABLED IFF DISABLE=True
 *
 * @param current_state The state the timer is in now
 * @return The next state the timer should be in
 */
TimerState Timer::getNextState(TimerState current_state)
{
  /// All states can and should transition to DISABLED.
  if (control_interface_.getPortMsg(DISABLE_).data)
    return TimerState::DISABLED;

  switch (current_state)
  {
    case TimerState::DISABLED:
    return control_interface_.getPortMsg(ENABLE_).data ? TimerState::IDLE : current_state;

    case TimerState::IDLE:
    return default_control_.getStartMsg().data ? TimerState::RUNNING : current_state;

    case TimerState::RUNNING:
    return default_control_.getStopMsg().data ? TimerState::FINISHED : current_state;

    case TimerState::FINISHED:
    return default_control_.getResetMsg().data ? TimerState::IDLE : current_state;
  }
}

/**
 * @brief Main calculator logic, called by the timer. Overrides AMBSBaseCalculator::executeCB()
 * @param event Not used
 */
void Timer::executeCB()
{
  while (ros::ok())
  {
    ros::Rate(100).sleep();
    next_state_ = getNextState(current_state_);
    // Any state -> DISABLED
    if (next_state_ == TimerState::DISABLED)
    {
      if (current_state_ != TimerState::DISABLED)
      {
        ROS_INFO_STREAM(node_name_ << ": DISABLED. Waiting for Enable.");
        if (debug_states_) debug_logger_.logInfo("DISABLED");
      }
      default_control_.resetAllPorts();
      float_interface_.resetAllPorts();
      control_interface_.resetAllPorts();
      current_state_ = TimerState::DISABLED;
    }
    // DISABLED -> IDLE
    else if (current_state_ == TimerState::DISABLED && next_state_ == TimerState::IDLE)
    {
      ROS_INFO_STREAM(node_name_ << ": IDLE. Waiting for Start.");
      current_state_ = TimerState::IDLE;
      if (debug_states_) debug_logger_.logInfo("IDLE");
    }
    // IDLE -> RUNNING
    else if (current_state_ == TimerState::IDLE && next_state_ == TimerState::RUNNING)
    {
       ROS_INFO_STREAM(node_name_ << ": RUNNING. Started timer for "
                      << getResolvedParam<float>(PARAM_DURATION_, default_duration_) << "s. Can interrupt.");
       start_time_ = ros::Time::now();
       current_state_ = TimerState::RUNNING;
       if (debug_states_) debug_logger_.logInfo("RUNNING");
    }
    // RUNNING -> RUNNING
    else if (current_state_ == TimerState::RUNNING && next_state_ == TimerState::RUNNING)
    {
       current_time_ = ros::Time::now();
       elapsed_ = current_time_ - start_time_;
       // RUNNING -> FINISHED - Timed out
       if ( elapsed_ >= ros::Duration(getResolvedParam<float>(PARAM_DURATION_, default_duration_)))
       {
          ROS_INFO_STREAM(node_name_ << ": FINISHED - Timed out. Waiting for reset.");
          std_msgs::Float64 msg;
          msg.data = elapsed_.toSec();
          control_interface_.publishMsgOnPort(TIMED_OUT_, control_interface_.constructNewBoolStamped(true));
          float_interface_.publishMsgOnPort(ELAPSED_, msg);
          default_control_.publishDone();
          current_state_ = TimerState::FINISHED;
          if (debug_states_) debug_logger_.logInfo("FINISHED");
       }
    }
    // RUNNING -> FINISHED - Interrupted
    else if (current_state_ == TimerState::RUNNING && next_state_ == TimerState::FINISHED)
    {
      ROS_INFO_STREAM(node_name_ << ": FINISHED - Interrupted. Elapsed "
                      << elapsed_.toSec() << "s. Waiting for Reset.");
      std_msgs::Float64 msg;
      msg.data = elapsed_.toSec();
      control_interface_.publishMsgOnPort(INTERRUPTED_, control_interface_.constructNewBoolStamped(true));
      float_interface_.publishMsgOnPort(ELAPSED_, msg);
      default_control_.publishDone();
      current_state_ = TimerState::FINISHED;
      if (debug_states_) debug_logger_.logInfo("FINISHED");
    }
    // FINISHED -> IDLE
    else if (current_state_ == TimerState::FINISHED && next_state_ == TimerState::IDLE)
    {
      ROS_INFO_STREAM(node_name_ << ": Resetting to IDLE. Waiting for Start.");
      default_control_.resetAllPorts();
      float_interface_.resetAllPorts();
      control_interface_.resetAllPorts();
      current_state_ = TimerState::IDLE;
      if (debug_states_) debug_logger_.logInfo("IDLE");
    }
  }
}

}  // namespace ambs_calculators


#endif  // AMBS_COMPONENTS_AMBS_CALCULATORS_DIFFERENTIATORS_TIMER_H
