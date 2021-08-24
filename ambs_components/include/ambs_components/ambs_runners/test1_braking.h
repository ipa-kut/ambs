#ifndef AMBS_COMPONENTS_AMBS_RUNNERS_TEST1_BRAKING_H
#define AMBS_COMPONENTS_AMBS_RUNNERS_TEST1_BRAKING_H

#include <string>
#include <vector>

#include "ambs_core/ambs_base_runner/ambs_base_runner.h"

namespace ambs_runners
{

/**
 * @brief Test 1 braking starts a robot and stops after it reaches max vel or times out
 */
class Test1Braking : public ambs_base::AMBSBaseRunner
{
public:
  Test1Braking() {}
  ~Test1Braking() override {}
  Test1Braking(ros::NodeHandle nh, std::string node_name):
     ambs_base::AMBSBaseRunner(nh, node_name)
  {}

  void executeCB(const ros::TimerEvent& event) override;
  void init();

private:
  const std::string PARAM_STABILIZATION_ = "stabilization_timeout";
  const std::string PARAM_ACCELERATION_ = "acceleration_timeout";
  const std::string PARAM_VERIFICATION_ = "verification_timeout";
  const std::string PARAM_DECCELERATION_ = "decceleration_timeout";

  const std::string ROBOT_IS_STATIONARY_ = "robot_is_stationary";
  const std::string ROBOT_HAS_MAX_VEL_ = "robot_has_max_vel";

  std::vector<std::string> extended_inputs_{ROBOT_IS_STATIONARY_, ROBOT_HAS_MAX_VEL_};
  std::vector<std::string> extended_outputs_;
  double param_stabilization_;
  double param_acceleration_;
  double param_verification_;
  double param_decceleration_;
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


/**
 * @brief Main runner logic, called by the timer. Overrides AMBSBaseRunner::executeCB()
 * @param event Not used
 */
void Test1Braking::executeCB(const ros::TimerEvent& event)
{
  (void) event;
  std::string result;

  /// Wait for start & rosbagger ready, then sleep to let robot initial jitter settle
  waitForStart();
  startRobot();
  ros::Duration(param_stabilization_).sleep();

  /// Wait some time to let robot reach max vel. If timeout, test fail.
  ROS_INFO_STREAM(node_name_ << ": Waiting " << param_acceleration_ <<"s for tobot to reach max vel");
  result = timedLoopFallbackOnPorts(new std::vector<std::string>{ROBOT_HAS_MAX_VEL_}, param_acceleration_);
  if (result == "")
  {
    ROS_WARN_STREAM(node_name_ << ": Robot did not reach max vel in expected time, ending test in failure!!");
    testFailed();
    stopRobot();
    waitForReset();
    return;
  }

  /// Wait some time and check again to see if robot maintained max vel
  ROS_INFO_STREAM(node_name_ << ": Robot hit max vel, waiting " << param_verification_ <<"s to recheck");
  ros::Duration(param_verification_).sleep();
  if (signal_interface_.getPortMsg(ROBOT_HAS_MAX_VEL_).data)
  {
    /// If so, then stop it
    ROS_INFO_STREAM(node_name_ << ": Robot maintained max vel, stopping it");
    stopRobot();
    // If it actually stops in a limited time, success, else failure
    result = timedLoopFallbackOnPorts(new std::vector<std::string>{ROBOT_IS_STATIONARY_}, param_decceleration_);
    if (result == ROBOT_IS_STATIONARY_)
    {
      ROS_WARN_STREAM(node_name_ << ": Robot has stopped, ending test in success!!");
      testSucceeded();
    }
    else
    {
      ROS_WARN_STREAM(node_name_ << ": Robot did not stop in " << param_decceleration_ <<"s, ending test in failure!!");
      testFailed();
    }
  }
  else
  {
    ROS_WARN_STREAM(node_name_ << ": Robot did not maintain max_vel, ending test in failure!!");
    testFailed();
  }

  waitForReset();
  return;
}

/**
 * @brief init function will be called by nodelet, so it is not called anywhere from within class
 *
 * AMBSBaseRunner::startRunner() spawns a timer which executes executeCB()
 */
void Test1Braking::init()
{
  ROS_INFO_STREAM(node_name_ << ": Init class");
  initialiseBaseRunner(nh_, node_name_, extended_inputs_, extended_outputs_);
  param_stabilization_ = getResolvedParam<double>(PARAM_STABILIZATION_);
  param_acceleration_ = getResolvedParam<double>(PARAM_ACCELERATION_);
  param_verification_ = getResolvedParam<double>(PARAM_VERIFICATION_);
  param_decceleration_ = getResolvedParam<double>(PARAM_DECCELERATION_);
  startRunner();
}


}  // namespace ambs_runners

#endif  // AMBS_COMPONENTS_AMBS_RUNNERS_TEST1_BRAKING_H
