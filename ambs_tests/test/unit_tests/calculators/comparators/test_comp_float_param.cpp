#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "ambs_msgs/BoolStamped.h"
#include "ambs_core/ambs_base_interface/ambs_boolean_interface.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"

double GLOBAL_PARAM;
double GLOBAL_TOLERANCE;

class TestCompFloatParam : public testing::Test
{
public:
  TestCompFloatParam() {}
  TestCompFloatParam(double param, double tolerance):
    param_(param),
    tolerance_(tolerance)
  {}

  const std::string START_ = "out_start";
  const std::string STOP_ = "out_stop";
  const std::string RESET_ = "out_reset";
  const std::string DONE_ = "in_done";
  const std::string FLOAT_ ="out_float";
  const std::string COMPARISON_ = "in_comparison";
  const std::string PARAM_ = "/ambs/calculators/test_comp_float_temporal/param";
  const std::string TOLERANCE_ = "/ambs/calculators/test_comp_float_temporal/tolerance";
  const double response_time_ = 0.25;
  const double spawn_time_ = 1;

  std::vector<std::string> bool_inputs_{DONE_, COMPARISON_};
  std::vector<std::string> bool_outputs_{START_, STOP_, RESET_};
  std::vector<std::string> float_inputs_;
  std::vector<std::string> float_outputs_{FLOAT_};

  ambs_base::AMBSBooleanInterface control_iface;
  ambs_base::AMBSTemplatedInterface<std_msgs::Float64> float_interface_;
  ros::NodeHandle nh_;
  double param_;
  double tolerance_;

  void init(ros::NodeHandle nh, double param, double tolerance);
};


void TestCompFloatParam::init(ros::NodeHandle nh, double param, double tolerance)
{
  nh_ = nh;

  control_iface.init(bool_inputs_, bool_outputs_, nh_, ros::this_node::getName());
  float_interface_.init(float_inputs_, float_outputs_, nh_, ros::this_node::getName());
  param_ = param;
  tolerance_ = tolerance;
}

TEST_F(TestCompFloatParam, test_comparison_true_false_true)
{
  ros::NodeHandle nh;
  init(nh, GLOBAL_PARAM, GLOBAL_TOLERANCE);

  // Waiting for tested nodelet to come up
  ros::Duration(spawn_time_).sleep();

  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time_).sleep();

  std_msgs::Float64 msg;
  msg.data = param_ + tolerance_/2;

  // Send True comparison
  float_interface_.publishMsgOnPort(FLOAT_, msg);
  ros::Duration(response_time_).sleep();

  // Check True comparison
  EXPECT_TRUE(control_iface.getPortMsg(COMPARISON_).data);

  ros::Duration(response_time_).sleep();

  msg.data = param_ + tolerance_ + 0.1;
  // Send False comparison
  float_interface_.publishMsgOnPort(FLOAT_, msg);
  ros::Duration(response_time_).sleep();

  // Check false comparison
  EXPECT_FALSE(control_iface.getPortMsg(COMPARISON_).data);

  msg.data = param_ - tolerance_/2;
  // Send True comparison
  float_interface_.publishMsgOnPort(FLOAT_, msg);
  ros::Duration(response_time_).sleep();

  // Check True comparison
  EXPECT_TRUE(control_iface.getPortMsg(COMPARISON_).data);

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

TEST_F(TestCompFloatParam, test_comparison_true_false_again)
{
  ros::NodeHandle nh;
  init(nh, GLOBAL_PARAM, GLOBAL_TOLERANCE);

  // Waiting for tested nodelet to come up
  ros::Duration(spawn_time_/2).sleep();
  control_iface.publishMsgOnPort(START_, control_iface.constructNewBoolStamped(true));
  ros::Duration(spawn_time_/2).sleep();

  std_msgs::Float64 msg;
  msg.data = param_ + tolerance_/2;

  // Send True comparison
  float_interface_.publishMsgOnPort(FLOAT_, msg);
  ros::Duration(response_time_).sleep();

  // Check True comparison
  EXPECT_TRUE(control_iface.getPortMsg(COMPARISON_).data);

  ros::Duration(response_time_).sleep();

  msg.data = param_ + tolerance_ + 0.1;
  // Send False comparison
  float_interface_.publishMsgOnPort(FLOAT_, msg);
  ros::Duration(response_time_).sleep();

  // Check false comparison
  EXPECT_FALSE(control_iface.getPortMsg(COMPARISON_).data);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_float_param_comp");

    GLOBAL_PARAM = atof(argv[1]);
    GLOBAL_TOLERANCE = atof(argv[2]);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}

