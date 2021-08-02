#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "ambs_msgs/BoolStamped.h"
#include "ambs_core/ambs_base_interface/ambs_boolean_interface.hpp"

class TestEdgeDetector : public testing::Test
{
public:
  TestEdgeDetector() {}

  const std::string START_ = "out_start";
  const std::string STOP_ = "out_stop";
  const std::string RESET_ = "out_reset";
  const std::string DONE_ = "in_done";
  const std::string BOOL_ ="out_bool";
  const std::string RISING_ = "in_rising";
  const std::string FALLING_ = "in_falling";
  const double response_time_ = 0.25;
  const double spawn_time_ = 1;

  std::vector<std::string> bool_inputs_{DONE_, RISING_, FALLING_};
  std::vector<std::string> bool_outputs_{START_, STOP_, RESET_, BOOL_};

  ambs_base::AMBSBooleanInterface control_iface;
  ros::NodeHandle nh_;

  void init(ros::NodeHandle nh);
};


void TestEdgeDetector::init(ros::NodeHandle nh)
{
  nh_ = nh;

  control_iface.init(bool_inputs_, bool_outputs_, nh_, ros::this_node::getName());
}

TEST_F(TestEdgeDetector, test_up_down_up)
{
  ros::NodeHandle nh;
  init(nh);

  // Waiting for tested nodelet to come up
  ros::Duration(spawn_time_).sleep();

  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  // Send True
  control_iface.publishMsgOnPort(BOOL_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  // Check Edges
  EXPECT_TRUE(control_iface.getPortMsg(RISING_).data);
  EXPECT_FALSE(control_iface.getPortMsg(FALLING_).data);


  // Send False
  control_iface.publishMsgOnPort(BOOL_, control_iface.constructNewBoolStamped(false));
  ros::Duration(response_time_).sleep();

  // Check Edges
  EXPECT_FALSE(control_iface.getPortMsg(RISING_).data);
  EXPECT_TRUE(control_iface.getPortMsg(FALLING_).data);


  // Send True
  control_iface.publishMsgOnPort(BOOL_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  // Check Edges
  EXPECT_TRUE(control_iface.getPortMsg(RISING_).data);
  EXPECT_FALSE(control_iface.getPortMsg(FALLING_).data);

  // Send stop
  ros::Duration(response_time_).sleep();

  control_iface.publishMsgOnPort(STOP_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_TRUE(control_iface.getPortMsg(DONE_).data);

  // Reset
  ros::Duration(spawn_time_/2).sleep();
  control_iface.publishMsgOnPort(RESET_, control_iface.constructNewBoolStamped(true));
  ros::Duration(spawn_time_/2).sleep();
}

TEST_F(TestEdgeDetector, test_down_up)
{
  ros::NodeHandle nh;
  init(nh);

  // Waiting for tested nodelet to come up
  ros::Duration(spawn_time_).sleep();

  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  // Send False
  control_iface.publishMsgOnPort(BOOL_, control_iface.constructNewBoolStamped(false));
  ros::Duration(response_time_).sleep();

  // Check Edges
  EXPECT_FALSE(control_iface.getPortMsg(RISING_).data);
  EXPECT_TRUE(control_iface.getPortMsg(FALLING_).data);


  // Send True
  control_iface.publishMsgOnPort(BOOL_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  // Check Edges
  EXPECT_TRUE(control_iface.getPortMsg(RISING_).data);
  EXPECT_FALSE(control_iface.getPortMsg(FALLING_).data);

  // Send stop
  ros::Duration(response_time_).sleep();

  control_iface.publishMsgOnPort(STOP_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();
  EXPECT_TRUE(control_iface.getPortMsg(DONE_).data);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_float_param_comp");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}

