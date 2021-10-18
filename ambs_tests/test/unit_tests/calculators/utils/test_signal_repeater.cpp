#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "ambs_msgs/BoolStamped.h"
#include "ambs_core/ambs_base_interface/ambs_boolean_interface.hpp"

class TestSignalRepeater : public testing::Test
{
public:
  TestSignalRepeater() {}

  const std::string START_ = "out_start";
  const std::string STOP_ = "out_stop";
  const std::string RESET_ = "out_reset";
  const std::string DONE_ = "in_done";
  const std::string IN_SIGNAL_ = "in_signal";
  const std::string OUT_SIGNAL_ = "out_signal";
  const double response_time_ = 0.1; // Signal repeater @20Hz = 0.05s delay to change, +0.05s for transmission
  const double spawn_time_ = 1;

  std::vector<std::string> OUT_SIGNAL_inputs_{DONE_, IN_SIGNAL_};
  std::vector<std::string> OUT_SIGNAL_outputs_{START_, STOP_, RESET_, OUT_SIGNAL_};

  ambs_base::AMBSBooleanInterface control_iface;
  ros::NodeHandle nh_;

  void init(ros::NodeHandle nh);
};


void TestSignalRepeater::init(ros::NodeHandle nh)
{
  nh_ = nh;

  control_iface.init(OUT_SIGNAL_inputs_, OUT_SIGNAL_outputs_, nh_, ros::this_node::getName());
}

TEST_F(TestSignalRepeater, test_one_pulse)
{
  ros::NodeHandle nh;
  init(nh);

  // Waiting for tested nodelet to come up
  ros::Duration(spawn_time_).sleep();

  ROS_INFO_STREAM(ros::this_node::getName() << ": Send start");
  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  // Send True
  ROS_INFO_STREAM(ros::this_node::getName() << ": Send True");
  control_iface.publishMsgOnPort(OUT_SIGNAL_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  // Get first response
  ambs_msgs::BoolStamped first = control_iface.getPortMsg(IN_SIGNAL_);
  ROS_INFO_STREAM(ros::this_node::getName() << ": Got first response- " << first);

  // Wait and get second response
  ros::Duration(response_time_).sleep();
  ambs_msgs::BoolStamped second = control_iface.getPortMsg(IN_SIGNAL_);
  ROS_INFO_STREAM(ros::this_node::getName() << ": Got second response- " << second);

  // First and second msgs must have same data but different stamps
  EXPECT_EQ(first.data, second.data);
  EXPECT_NE(first.header.seq, second.header.seq);
  EXPECT_NE(first.header.stamp, second.header.stamp);

  // Next Send False
  ros::Duration(response_time_).sleep();
  control_iface.publishMsgOnPort(OUT_SIGNAL_, control_iface.constructNewBoolStamped(false));
  ros::Duration(response_time_).sleep();

  // Get first response
  first = control_iface.getPortMsg(IN_SIGNAL_);

  // Wait and get second response
  ros::Duration(response_time_).sleep();
  second = control_iface.getPortMsg(IN_SIGNAL_);

  // First and second msgs must have same data but different stamps
  EXPECT_EQ(first.data, second.data);
  EXPECT_NE(first.header.seq, second.header.seq);
  EXPECT_NE(first.header.stamp, second.header.stamp);

  // Reset
  ros::Duration(spawn_time_/2).sleep();
  control_iface.publishMsgOnPort(STOP_, control_iface.constructNewBoolStamped(true));
  control_iface.publishMsgOnPort(RESET_, control_iface.constructNewBoolStamped(true));
  ros::Duration(spawn_time_/2).sleep();
}

TEST_F(TestSignalRepeater, test_another_pulse_after_reset)
{
  ros::NodeHandle nh;
  init(nh);

  // Waiting for tested nodelet to come up
  ros::Duration(spawn_time_/2).sleep();

  ROS_INFO_STREAM(ros::this_node::getName() << ": Send start");
  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  // Send True
  ROS_INFO_STREAM(ros::this_node::getName() << ": Send True");
  control_iface.publishMsgOnPort(OUT_SIGNAL_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  // Get first response
  ambs_msgs::BoolStamped first = control_iface.getPortMsg(IN_SIGNAL_);
  ROS_INFO_STREAM(ros::this_node::getName() << ": Got first response- " << first);

  // Wait and get second response
  ros::Duration(response_time_).sleep();
  ambs_msgs::BoolStamped second = control_iface.getPortMsg(IN_SIGNAL_);
  ROS_INFO_STREAM(ros::this_node::getName() << ": Got second response- " << second);

  // First and second msgs must have same data but different stamps
  EXPECT_EQ(first.data, second.data);
  EXPECT_NE(first.header.seq, second.header.seq);
  EXPECT_NE(first.header.stamp, second.header.stamp);

  // Next Send False
  ros::Duration(response_time_).sleep();
  control_iface.publishMsgOnPort(OUT_SIGNAL_, control_iface.constructNewBoolStamped(false));
  ros::Duration(response_time_).sleep();

  // Get first response
  first = control_iface.getPortMsg(IN_SIGNAL_);

  // Wait and get second response
  ros::Duration(response_time_).sleep();
  second = control_iface.getPortMsg(IN_SIGNAL_);

  // First and second msgs must have same data but different stamps
  EXPECT_EQ(first.data, second.data);
  EXPECT_NE(first.header.seq, second.header.seq);
  EXPECT_NE(first.header.stamp, second.header.stamp);

  // Reset
  ros::Duration(spawn_time_/2).sleep();
  control_iface.publishMsgOnPort(STOP_, control_iface.constructNewBoolStamped(true));
  control_iface.publishMsgOnPort(RESET_, control_iface.constructNewBoolStamped(true));
  ros::Duration(spawn_time_/2).sleep();
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_signal_booster");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}

