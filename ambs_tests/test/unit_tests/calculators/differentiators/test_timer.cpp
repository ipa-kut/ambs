#include <gtest/gtest.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "ambs_msgs/BoolStamped.h"
#include "ambs_core/ambs_base_interface/ambs_boolean_interface.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "ambs_core/ambs_helper/helper.h"

double GLOBAL_DURATION;

class TestTimer : public testing::Test
{
public:
  TestTimer() {}

  void diagCB(const diagnostic_msgs::DiagnosticArray::ConstPtr msg);
  ros::Subscriber debug_sub_;
  boost::shared_ptr<std::string> log_msg_;

  const std::string START_ = "out_start";
  const std::string STOP_ = "out_stop";
  const std::string RESET_ = "out_reset";
  const std::string ENABLE_ = "out_enable";
  const std::string DISABLE_ = "out_disable";
  const std::string DONE_ = "in_done";
  const std::string ELAPSED_ = "in_elapsed";
  const std::string INTERRUPTED_ = "in_interrupted";
  const std::string TIMED_OUT_ = "in_timed_out";

  const double response_time_ = 0.05;
  const double spawn_time_ = 0.5;

  std::vector<std::string> bool_inputs_{DONE_, INTERRUPTED_, TIMED_OUT_};
  std::vector<std::string> bool_outputs_{START_, STOP_, RESET_, ENABLE_, DISABLE_};
  std::vector<std::string> float_inputs_{ELAPSED_};
  std::vector<std::string> float_outputs_;

  ambs_base::AMBSBooleanInterface control_iface;
  ambs_base::AMBSTemplatedInterface<std_msgs::Float64> float_interface_;
  ros::NodeHandle nh_;

  void init(ros::NodeHandle nh);
};


void TestTimer::diagCB(const diagnostic_msgs::DiagnosticArray::ConstPtr msg)
{
  // Specific to this test only. Assuming that the first debug msg is from the tested timer.
  *log_msg_ = msg->status[0].message;
}

void TestTimer::init(ros::NodeHandle nh)
{
  nh_ = nh;

  control_iface.init(bool_inputs_, bool_outputs_, nh_, ros::this_node::getName());
  float_interface_.init(float_inputs_, float_outputs_, nh_, ros::this_node::getName());
  log_msg_.reset(new std::string());
  debug_sub_ =
      nh_.subscribe("/ambs/debug_messages",1000,&TestTimer::diagCB, this);
}

TEST_F(TestTimer, test_forward_and_disable)
{
  ros::NodeHandle nh;
  init(nh);

  // Waiting for tested nodelet to come up
  ros::Duration(spawn_time_).sleep();

  // --- PART 1 -- Disable while Idle
  // 1. DISABLED -> IDLE
  control_iface.publishMsgOnPort(ENABLE_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("IDLE", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // 1. IDLE -> DISABLED
  control_iface.publishMsgOnPort(DISABLE_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("DISABLED", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // ------PART2 -- Disable while Running
  // 2. DISABLED -> IDLE
  control_iface.publishMsgOnPort(ENABLE_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("IDLE", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // 2. IDLE -> RUNNING
  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("RUNNING", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // 2. RUNNING -> DISABLED
  control_iface.publishMsgOnPort(DISABLE_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("DISABLED", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // ------PART3 -- Disable while Finished
  // 3. DISABLED -> IDLE
  control_iface.publishMsgOnPort(ENABLE_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("IDLE", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // 3. IDLE -> RUNNING
  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("RUNNING", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // 3. RUNNING -> FINISHED
  control_iface.publishMsgOnPort(STOP_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("FINISHED", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // 3. FINISHED -> DISABLED
  control_iface.publishMsgOnPort(DISABLE_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("DISABLED", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);
}

TEST_F(TestTimer, test_timeout_reset_timeout)
{
  ros::NodeHandle nh;
  init(nh);

  // Waiting for tested nodelet to come up
  ros::Duration(spawn_time_).sleep();

  // DISABLED -> IDLE
  control_iface.publishMsgOnPort(ENABLE_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("IDLE", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // IDLE -> RUNNING
  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("RUNNING", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // Wait for and verify FINISHED - Timed out
  ROS_INFO_STREAM("Wait" << GLOBAL_DURATION << "s for timeout.");
  ros::Duration(GLOBAL_DURATION).sleep();
  EXPECT_EQ("FINISHED", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);
  EXPECT_TRUE(control_iface.getPortMsg(DONE_).data);
  EXPECT_TRUE(control_iface.getPortMsg(TIMED_OUT_).data);
  EXPECT_LE(float_interface_.getPortMsg(ELAPSED_).data, GLOBAL_DURATION + 0.05); // 0.05s delay padding

  // FINISHED -> IDLE
  control_iface.publishMsgOnPort(RESET_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("IDLE", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // IDLE -> RUNNING
  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("RUNNING", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // Wait for and verify FINISHED - Timed out
  ROS_INFO_STREAM("Wait" << GLOBAL_DURATION << "s for timeout.");
  ros::Duration(GLOBAL_DURATION).sleep();
  EXPECT_EQ("FINISHED", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);
  EXPECT_TRUE(control_iface.getPortMsg(DONE_).data);
  EXPECT_TRUE(control_iface.getPortMsg(TIMED_OUT_).data);
  EXPECT_LE(float_interface_.getPortMsg(ELAPSED_).data, GLOBAL_DURATION + 0.05); // 0.05s delay padding

  // FINISHED -> DISABLED
  control_iface.publishMsgOnPort(DISABLE_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("DISABLED", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);
}

TEST_F(TestTimer, test_interrupt_timeout)
{
  ros::NodeHandle nh;
  init(nh);

  // Waiting for tested nodelet to come up
  ros::Duration(spawn_time_).sleep();

  // DISABLED -> IDLE
  control_iface.publishMsgOnPort(ENABLE_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("IDLE", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // IDLE -> RUNNING
  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("RUNNING", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);

  // RUNNING -> FINISHED - Interrupted
  // Wait half the timer duration to interrupt
  ros::Duration(GLOBAL_DURATION/2).sleep();
  ROS_INFO_STREAM("Interrupt! after " << GLOBAL_DURATION/2 << "s");
  control_iface.publishMsgOnPort(STOP_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("FINISHED", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);
  EXPECT_TRUE(control_iface.getPortMsg(DONE_).data);
  EXPECT_TRUE(control_iface.getPortMsg(INTERRUPTED_).data);
  // Elapsed time could be a little more than the calculated pause, but never more than time out
  EXPECT_LE(float_interface_.getPortMsg(ELAPSED_).data, GLOBAL_DURATION);
  EXPECT_GE(float_interface_.getPortMsg(ELAPSED_).data, GLOBAL_DURATION/2);

  // FINISHED -> IDLE
  control_iface.publishMsgOnPort(RESET_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_EQ("IDLE", *log_msg_);
  ROS_INFO_STREAM("Response - " << *log_msg_);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_timer");

    GLOBAL_DURATION = atof(argv[1]);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    return ret;
}

