#ifndef AMBS_COMPONENTS_AMBS_CALCULATORS_DIFFERENTIATORS_DIFF_POSE_TEMPORAL_H
#define AMBS_COMPONENTS_AMBS_CALCULATORS_DIFFERENTIATORS_DIFF_POSE_TEMPORAL_H

#include <string>
#include <vector>

#include "ambs_core/ambs_base_calculator/ambs_base_calculator.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "ambs_core/ambs_helper/helper.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>


namespace ambs_calculators
{

/**
 * @brief Compute difference in a pose value from a topic between start and stop
 *
 * Has standard control intetrface, a pose interface with one input and a float interface with one output port
 */
class DiffPoseTemporal : public ambs_base::AMBSBaseCalculator
{
public:
  DiffPoseTemporal() {}
  ~DiffPoseTemporal() {}
  DiffPoseTemporal(ros::NodeHandle nh, std::string node_name):
    ambs_base::AMBSBaseCalculator(nh, node_name)
  {}
  void init();

private:
  void executeCB(const ros::TimerEvent& event) override;
  void printPose(geometry_msgs::PoseStamped pose);

  ambs_base::AMBSTemplatedInterface<std_msgs::Float64> float_interface_;
  ambs_base::AMBSTemplatedInterface<geometry_msgs::PoseStamped> pose_interface_;
  std_msgs::Float64 result_msg_;
  const std::string DIFF_POSITION_ = "/out_diff_position";
  const std::string DIFF_ORIENTATION = "/out_diff_orientation";
  const std::string IN_POSE_ = "/in_pose";
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


/**
 * @brief init function will be called by nodelet, so it is not called anywhere from within class
 *
 * AMBSBaseCalculator::startCalculator() spawns a timer which executes executeCB()
 */
void DiffPoseTemporal::init()
{
  ROS_INFO_STREAM(node_name_ <<": Init class");

  std::vector<std::string> float_inputs{};
  std::vector<std::string> float_outputs{DIFF_POSITION_, DIFF_ORIENTATION};
  float_interface_.init(float_inputs, float_outputs, nh_, node_name_);

  std::vector<std::string> pose_inputs{IN_POSE_};
  std::vector<std::string> pose_outputs{};
  pose_interface_.init(pose_inputs, pose_outputs, nh_, node_name_);

  startCalculator();
}

/**
 * @brief Main calculator logic, called by the timer. Overrides AMBSBaseCalculator::executeCB()
 * @param event Not used
 */
void DiffPoseTemporal::executeCB(const ros::TimerEvent& event)
{
  (void) event;

  std_msgs::Float64 diff_pos_msg;
  std_msgs::Float64 diff_orientation_msg;

  default_control_.waitForStart();
  geometry_msgs::PoseStamped start_pose = pose_interface_.getPortMsg(IN_POSE_);
  ROS_INFO_STREAM(node_name_ << ": Starting pose stored: ");
  printPose(start_pose);
  ROS_INFO(" ");

  default_control_.waitForStop();
  geometry_msgs::PoseStamped stop_pose = pose_interface_.getPortMsg(IN_POSE_);
  ROS_INFO_STREAM(node_name_ << ": Stopping pose stored: ");
  printPose(stop_pose);
  ROS_INFO(" ");

  diff_pos_msg.data = ambs_helper::getTranslationDiffFromPoses(start_pose, stop_pose);
  diff_orientation_msg.data = ambs_helper::getYawDiffFromPoses(start_pose, stop_pose);

  ROS_INFO_STREAM(node_name_ << ": Diff Pos: " << diff_pos_msg.data << " Diff Ori: " << diff_orientation_msg.data);
  float_interface_.publishMsgOnPort(DIFF_POSITION_, diff_pos_msg);
  float_interface_.publishMsgOnPort(DIFF_ORIENTATION, diff_orientation_msg);
  default_control_.publishDone();

  default_control_.waitForReset();
  float_interface_.resetAllPorts();
  pose_interface_.resetAllPorts();

  ROS_INFO_STREAM(node_name_ << ": Restarting calculator");
  ROS_INFO(" ");
  ROS_INFO("-----------------------------------------------------------");
  ROS_INFO(" ");
}

/**
 * @brief Print pose values. Debug only.
 * @param pose Pose to print
 */
void DiffPoseTemporal::printPose(geometry_msgs::PoseStamped pose)
{
  ROS_INFO_STREAM(node_name_ << ": (" << pose.pose.position.x << ", "
                                      << pose.pose.position.y << ", "
                                      << pose.pose.position.z << ") ("
                                      << ambs_helper::getYawDegreesFromQuaternion(pose.pose.orientation) << " deg)");
}


}  // namespace ambs_calculators

#endif  // AMBS_COMPONENTS_AMBS_CALCULATORS_DIFFERENTIATORS_DIFF_POSE_TEMPORAL_H
