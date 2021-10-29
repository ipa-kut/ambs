#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "ambs_msgs/BoolStamped.h"
#include "ambs_core/ambs_base_interface/ambs_boolean_interface.hpp"

int GLOBAL_N_INPUTS;

class TestOrGate : public testing::Test
{
public:
  TestOrGate() {}

  const std::string START_ = "out_start";
  const std::string STOP_ = "out_stop";
  const std::string RESET_ = "out_reset";
  const std::string DONE_ = "in_done";
  const std::string IN_RESULT_ = "in_result";
  const std::string OUT_SIGNAL_ = "out_signal";
  const double response_time_ = 0.05;  // Signal repeater @20Hz = 0.05s delay to change, +0.05s for transmission
  const double spawn_time_ = 0.5;


  std::vector<std::string> control_inputs_{DONE_, IN_RESULT_};
  std::vector<std::string> control_outputs_{START_, STOP_, RESET_, OUT_SIGNAL_};

  ambs_base::AMBSBooleanInterface control_iface;
  ros::NodeHandle nh_;

  void init(ros::NodeHandle nh);
};


void TestOrGate::init(ros::NodeHandle nh)
{
  nh_ = nh;

  for (int i = 1; i <= GLOBAL_N_INPUTS; i++)
  {
    control_outputs_.push_back("test_or_gate/in_input_"+std::to_string(i));
  }

  control_iface.init(control_inputs_, control_outputs_, nh_, ros::this_node::getName());
}

TEST_F(TestOrGate, test_each_or_input)
{
  ros::NodeHandle nh;
  init(nh);

  // Waiting for tested nodelet to come up
  ros::Duration(spawn_time_).sleep();

  ROS_INFO_STREAM(ros::this_node::getName() << ": Send start");
  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  // 0 1 2 3 are START_, STOP_, RESET_, OUT_SIGNAL_
  for (int i = 4; i < control_outputs_.size(); i++)
  {
    std::string port = control_outputs_.at(i);
    ROS_INFO_STREAM(ros::this_node::getName() << ": Testing OR input " << port);
    control_iface.publishMsgOnPort(port, control_iface.constructNewBoolStamped(true));
    ros::Duration(response_time_).sleep();
    EXPECT_TRUE(control_iface.getPortMsg(IN_RESULT_).data);
  }

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

    GLOBAL_N_INPUTS = atoi(argv[1]);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}

