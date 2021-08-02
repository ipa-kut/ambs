#include <gtest/gtest.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>

#include "ambs_msgs/BoolStamped.h"
#include "ambs_core/ambs_base_interface/ambs_boolean_interface.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "ambs_core/ambs_helper/helper.h"

class TestStopwatch : public testing::Test
{
public:
  TestStopwatch() {}

  const std::string START_ = "out_start";
  const std::string STOP_ = "out_stop";
  const std::string RESET_ = "out_reset";
  const std::string DONE_ = "in_done";
  const std::string FLOAT_ = "in_float";
  const double response_time_ = 0.2;
  const double spawn_time_ = 2;
  const double test_sleep_time = 0.5;

  std::vector<std::string> bool_inputs_{DONE_};
  std::vector<std::string> bool_outputs_{START_, STOP_, RESET_};
  std::vector<std::string> float_inputs_{FLOAT_};
  std::vector<std::string> float_outputs_;

  ambs_base::AMBSBooleanInterface control_iface;
  ambs_base::AMBSTemplatedInterface<std_msgs::Float64> float_interface_;
  ros::NodeHandle nh_;

  void init(ros::NodeHandle nh);
};


void TestStopwatch::init(ros::NodeHandle nh)
{
  nh_ = nh;

  control_iface.init(bool_inputs_, bool_outputs_, nh_, ros::this_node::getName());
  float_interface_.init(float_inputs_, float_outputs_, nh_, ros::this_node::getName());
}

TEST_F(TestStopwatch, test_diff_time_once)
{
  ros::NodeHandle nh;
  init(nh);

  // Waiting for tested nodelet to come up
  ros::Duration(spawn_time_).sleep();
  ambs_msgs::BoolStamped start = control_iface.constructNewBoolStamped(true);
  control_iface.publishMsgOnPort(START_, start);

  ros::Duration(test_sleep_time).sleep();

  ambs_msgs::BoolStamped stop = control_iface.constructNewBoolStamped(true);
  control_iface.publishMsgOnPort(STOP_, stop);
  ros::Duration(response_time_).sleep();

  EXPECT_FLOAT_EQ(float_interface_.getPortMsg(FLOAT_).data,
                  (stop.header.stamp - start.header.stamp).toSec());
  EXPECT_TRUE(control_iface.getPortMsg(DONE_).data);
}

TEST_F(TestStopwatch, test_diff_time_again)
{
  ros::NodeHandle nh;
  init(nh);

  ros::Duration(spawn_time_/2).sleep();
  control_iface.publishMsgOnPort(RESET_, control_iface.constructNewBoolStamped(true));
  ros::Duration(spawn_time_/2).sleep();

  ambs_msgs::BoolStamped start = control_iface.constructNewBoolStamped(true);
  control_iface.publishMsgOnPort(START_, start);

  ros::Duration(test_sleep_time/2).sleep();

  ambs_msgs::BoolStamped stop = control_iface.constructNewBoolStamped(true);
  control_iface.publishMsgOnPort(STOP_, stop);
  ros::Duration(response_time_).sleep();

  EXPECT_FLOAT_EQ(float_interface_.getPortMsg(FLOAT_).data,
                  (stop.header.stamp - start.header.stamp).toSec());
  EXPECT_TRUE(control_iface.getPortMsg(DONE_).data);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_POSE_param_comp");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    return ret;
}

