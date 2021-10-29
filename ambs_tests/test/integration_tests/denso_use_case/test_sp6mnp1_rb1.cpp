#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <fstream>

#include "ambs_msgs/BoolStamped.h"
#include "ambs_core/ambs_base_interface/ambs_boolean_interface.hpp"
#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"

double G_OPEN1_ON;
double G_OPEN2_ON;
double G_CLOSE1_ON;
double G_CLOSE2_ON;
double G_PR1_ON;
double G_PR2_ON;

class TestSP6MNP1RB1 : public testing::Test
{
public:
  TestSP6MNP1RB1() {}
  TestSP6MNP1RB1(double param, double tolerance):
    param_(param),
    tolerance_(tolerance)
  {}

  const std::string OPEN1_ = "out_open1_busy";
  const std::string OPEN2_ = "out_open2_busy";
  const std::string CLOSE1_ = "out_close1_busy";
  const std::string CLOSE2_ = "out_close2_busy";
  const std::string PR1_ = "out_pr1_busy";
  const std::string PR2_ = "out_pr2_busy";
  const std::string START_ROBOT_ = "out_start_robot";
  const std::string STOP_ROBOT_ = "out_stop_robot";
  const std::string BAG_STARTED_ = "in_rosbag_began";

  const double response_time = 0.5;
  const double pulse_off_time_ = 0.1;

  std::string log_folder_;
  std::string csv_name_;

  std::vector<std::string> bool_inputs_{BAG_STARTED_};
  std::vector<std::string> bool_outputs_{OPEN1_, OPEN2_, CLOSE1_, CLOSE2_, PR1_, PR2_, START_ROBOT_, STOP_ROBOT_};

  ambs_base::AMBSBooleanInterface control_iface;
  ambs_base::AMBSTemplatedInterface<std_msgs::Float64> float_interface_;
  ros::NodeHandle nh_;
  double param_;
  double tolerance_;

  void init(ros::NodeHandle nh);
  void sendPulseOn(std::string port, double on_duration);
  int getNumberOfRowsInDebugCSV();
};


void TestSP6MNP1RB1::init(ros::NodeHandle nh)
{
  nh_ = nh;
  control_iface.init(bool_inputs_, bool_outputs_, nh_, ros::this_node::getName());
}

void TestSP6MNP1RB1::sendPulseOn(std::string port, double on_duration)
{
  //  Expecting duration from argv to be in miliseconds
  ROS_INFO_STREAM(ros::this_node::getName() << ": Pulse " << port << " on for " << on_duration/1000);
  ros::Duration(pulse_off_time_/2).sleep();
  control_iface.publishMsgOnPort(port, control_iface.constructNewBoolStamped(true));
  ros::Duration(on_duration/1000).sleep();
  control_iface.publishMsgOnPort(port, control_iface.constructNewBoolStamped(false));
  ros::Duration(pulse_off_time_/2).sleep();
}

int TestSP6MNP1RB1::getNumberOfRowsInDebugCSV()
{
  std::string s;
  int sTotal = 0;

  std::ifstream in;
  ROS_INFO_STREAM(ros::this_node::getName() << ": Opening file " << csv_name_);
  in.open(csv_name_);

  while (!in.eof())
  {
      getline(in, s);
      ROS_INFO_STREAM(ros::this_node::getName() << ": CSV: " << sTotal << " - " << s);
      sTotal++;
      if (sTotal > 20) break;
  }
  return sTotal-1;  // This always reads one extra line
}

TEST_F(TestSP6MNP1RB1, test_custom_pulse_train)
{
  ros::NodeHandle nh;
  init(nh);

  // Wait until tosbag starts, meaning everything should be up and running, and log folder param is available
  control_iface.waitForTrueOnPort(BAG_STARTED_);
  EXPECT_TRUE(nh_.getParam("/ambs/log_folder", log_folder_));
  csv_name_ = log_folder_ + "/debug.csv";

  // Start all calcs and wait for them to be ready
  control_iface.publishMsgOnPort(START_ROBOT_, control_iface.constructNewBoolStamped(true));
  ros::Duration(response_time).sleep();

  // Send a custom pulse train
  sendPulseOn(PR1_, G_PR1_ON);
  sendPulseOn(PR2_, G_PR2_ON);
  sendPulseOn(OPEN1_, G_OPEN1_ON);
  sendPulseOn(OPEN2_, G_OPEN2_ON);
  sendPulseOn(CLOSE1_, G_CLOSE1_ON);
  sendPulseOn(CLOSE2_, G_CLOSE2_ON);

  ///  @todo: Automate this test. i.e. Figure out file write/read for CI.
  ROS_INFO_STREAM(ros::this_node::getName() << ": CSV had "
                  << getNumberOfRowsInDebugCSV()-1 << " out of 6 expected result lines");
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_sp6mnp1_rb1");

    G_OPEN1_ON = atof(argv[1]);
    G_OPEN2_ON = atof(argv[2]);
    G_CLOSE1_ON = atof(argv[3]);
    G_CLOSE2_ON = atof(argv[4]);
    G_PR1_ON = atof(argv[5]);
    G_PR2_ON = atof(argv[6]);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}

