#include <gtest/gtest.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>

#include "ambs_msgs/BoolStamped.h"
#include "ambs_core/ambs_base_interface/ambs_boolean_interface.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"
#include "ambs_core/ambs_helper/helper.h"

class TestDiffFloatTemporal : public testing::Test
{
public:
  TestDiffFloatTemporal() {}

  const std::string START_ = "out_start";
  const std::string STOP_ = "out_stop";
  const std::string RESET_ = "out_reset";
  const std::string DONE_ = "in_done";
  const std::string FLOAT_IN_ = "in_float";
  const std::string FLOAT_OUT_ = "out_float";
  const double response_time_ = 0.25;
  const double spawn_time_ = 1;

  std::vector<std::string> bool_inputs_{DONE_};
  std::vector<std::string> bool_outputs_{START_, STOP_, RESET_};
  std::vector<std::string> float_inputs_{FLOAT_IN_};
  std::vector<std::string> float_outputs_{FLOAT_OUT_};

  ambs_base::AMBSBooleanInterface control_iface;
  ambs_base::AMBSTemplatedInterface<std_msgs::Float64> float_interface_;
  ros::NodeHandle nh_;

  void init(ros::NodeHandle nh);
};


void TestDiffFloatTemporal::init(ros::NodeHandle nh)
{
  nh_ = nh;

  control_iface.init(bool_inputs_, bool_outputs_, nh_, ros::this_node::getName());
  float_interface_.init(float_inputs_, float_outputs_, nh_, ros::this_node::getName());
}

TEST_F(TestDiffFloatTemporal, test_float_diff_once)
{
  ros::NodeHandle nh;
  init(nh);

  // Waiting for tested nodelet to come up
  ros::Duration(spawn_time_).sleep();

  std_msgs::Float64 start_float;
  start_float.data = 1.23;
  float_interface_.publishMsgOnPort(FLOAT_OUT_, start_float);
  ros::Duration(response_time_).sleep();
  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  std_msgs::Float64 stop_float;
  stop_float.data = 8.435;
  float_interface_.publishMsgOnPort(FLOAT_OUT_, stop_float);
  ros::Duration(response_time_).sleep();
  control_iface.publishMsgOnPort(STOP_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  EXPECT_FLOAT_EQ(float_interface_.getPortMsg(FLOAT_IN_).data,
                  stop_float.data - start_float.data);
  EXPECT_TRUE(control_iface.getPortMsg(DONE_).data);
}

TEST_F(TestDiffFloatTemporal, test_float_diff_again)
{
  ros::NodeHandle nh;
  init(nh);

  ros::Duration(spawn_time_/2).sleep();
  control_iface.publishMsgOnPort(RESET_, control_iface.constructNewBoolStamped(true));
  ros::Duration(spawn_time_/2).sleep();

  std_msgs::Float64 start_float;
  start_float.data = -145.2433;
  float_interface_.publishMsgOnPort(FLOAT_OUT_, start_float);
  ros::Duration(response_time_).sleep();
  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  std_msgs::Float64 stop_float;
  stop_float.data = -83456.435456;
  float_interface_.publishMsgOnPort(FLOAT_OUT_, stop_float);
  ros::Duration(response_time_).sleep();
  control_iface.publishMsgOnPort(STOP_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  EXPECT_FLOAT_EQ(float_interface_.getPortMsg(FLOAT_IN_).data,
                  stop_float.data - start_float.data);
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

