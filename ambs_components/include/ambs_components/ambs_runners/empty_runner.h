#ifndef EMPTY_RUNNER_H
#define EMPTY_RUNNER_H

#include <string>
#include <vector>

#include "ambs_core/ambs_base_runner/ambs_base_runner.h"

namespace ambs_runners
{

/**
 * @brief Test 1 braking starts a robot and stops after it reaches max vel or times out
 */
class EmptyRunner : public ambs_base::AMBSBaseRunner
{
public:
  EmptyRunner() {}
  ~EmptyRunner() override {}
  EmptyRunner(ros::NodeHandle nh, std::string node_name):
     ambs_base::AMBSBaseRunner(nh, node_name)
  {}

  void executeCB(const ros::TimerEvent& event) override;
  void init();
private:
  std::vector<std::string> extended_inputs_;
  std::vector<std::string> extended_outputs_;
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


/**
 * @brief Main runner logic, called by the timer. Overrides AMBSBaseRunner::executeCB()
 * @param event Not used
 */
void EmptyRunner::executeCB(const ros::TimerEvent& event)
{

}

/**
 * @brief init function will be called by nodelet, so it is not called anywhere from within class
 *
 * AMBSBaseRunner::startRunner() spawns a timer which executes executeCB()
 */
void EmptyRunner::init()
{
  ROS_INFO_STREAM(node_name_ << ": Init class");
  initialiseBaseRunner(nh_, node_name_, extended_inputs_, extended_outputs_);

  startRunner();
}


}  // namespace ambs_runners


#endif // EMPTY_RUNNER_H
