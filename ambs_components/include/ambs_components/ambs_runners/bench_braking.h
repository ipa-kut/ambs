#ifndef AMBS_COMPONENTS_AMBS_RUNNERS_BENCH_BRAKING_H
#define AMBS_COMPONENTS_AMBS_RUNNERS_BENCH_BRAKING_H

#include <string>
#include <vector>
#include <numeric>
#include <angles/angles.h>
#include <experimental/random>
#include <fstream>
#include <cmath>
#include <chrono>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include "ambs_core/ambs_base_runner/ambs_base_runner.h"
#include "ambs_components/ambs_loggers/debug_logger.h"
#include "ambs_core/ambs_helper/helper.h"

namespace ambs_runners
{

/**
 * @brief Benchmark braking test: analyse braking distance
 */
class BenchBraking : public ambs_base::AMBSBaseRunner
{
public:
  BenchBraking() {}
  ~BenchBraking() override {}
  BenchBraking(ros::NodeHandle nh, std::string node_name):
     ambs_base::AMBSBaseRunner(nh, node_name)
  {}

  void executeCB(const ros::TimerEvent& event) override;
  void mockRobot(const ros::TimerEvent& event);
  void init();

private:
  const std::string PARAM_STABILIZATION_ = "stabilization_timeout";
  const std::string PARAM_ACCELERATION_ = "acceleration_timeout";
  const std::string PARAM_VERIFICATION_ = "verification_timeout";
  const std::string PARAM_DECCELERATION_ = "decceleration_timeout";

  const std::string ROBOT_IS_STATIONARY_ = "robot_is_stationary";
  const std::string ROBOT_HAS_MAX_VEL_ = "robot_has_max_vel";

  const std::string CALC_BRAKING_DISTANCE_ = "in_calc_braking_distance";
  const std::string ROBOT_LOCATION = "out_robot_location";
  const std::string ROBOT_SPEED = "out_robot_speed_x";
  const std::string RESET_TEST = "out_reset_test";

  std::vector<std::string> extended_inputs_{ROBOT_IS_STATIONARY_, ROBOT_HAS_MAX_VEL_};
  std::vector<std::string> extended_outputs_{RESET_TEST};

  ambs_base::AMBSTemplatedInterface<geometry_msgs::PoseStamped> pose_interface_;
  std::vector<std::string> pose_inputs_;
  std::vector<std::string> pose_outputs_{ROBOT_LOCATION};

  ambs_base::AMBSTemplatedInterface<std_msgs::Float64> float_interface_;
  std::vector<std::string> float_inputs_{CALC_BRAKING_DISTANCE_};
  std::vector<std::string> float_outputs_{ROBOT_SPEED};

  ros::Timer mock_robot_;
  ambs_loggers::DebugLogger debug_logger_;

  int loops_;
  int sensor_rate_;

  double getStandardDeviation(std::vector<double> data);
  double getMaxDeviation(std::vector<double> data);
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


/**
 * @brief Mocks the robot & sensor: provides robot speed & sensor location
 * @param event Ignored
 */
void BenchBraking::mockRobot(const ros::TimerEvent &event)
{
  (void) event;
  int x = 0;

  while (ros::ok())
  {
    if (sin(angles::from_degrees(x)) == 0) continue;
    geometry_msgs::PoseStamped msg_pos;
    msg_pos.header.stamp = ros::Time::now();
    msg_pos.pose.position.x = sin(angles::from_degrees(x));
    msg_pos.pose.position.y = 0;
    msg_pos.pose.position.z = 0;
    msg_pos.pose.orientation.x = 0;
    msg_pos.pose.orientation.y = 0;
    msg_pos.pose.orientation.z = 0;
    msg_pos.pose.orientation.w = 1;
    pose_interface_.publishMsgOnPort(ROBOT_LOCATION, msg_pos);

    std_msgs::Float64 msg_spd;
    msg_spd.data = sin(angles::from_degrees(x));
    float_interface_.publishMsgOnPort(ROBOT_SPEED, msg_spd);

//    Randomize next run
    x = (x == 360) ? std::experimental::randint(0, 50) : x+1;
    ros::Rate(sensor_rate_).sleep();
    ros::spinOnce();
  }
}


/**
 * @brief Main runner logic, called by the timer. Overrides AMBSBaseRunner::executeCB()
 * @param event Not used
 */
void BenchBraking::executeCB(const ros::TimerEvent& event)
{
  (void) event;
  std::vector<double> error_list;
  std::vector<double> accuracy_list;
  double error, accuracy, err_avg, acc_avg;

  ros::Duration(1).sleep();
  ROS_WARN_STREAM(node_name_ << "Running bench for " << loops_ << " loops @ "
                  << sensor_rate_ <<"Hz sensor rate");
  std::ofstream debug_file;
  debug_file.open(log_folder_path_ + "/benchmarking.csv", std::ios_base::app);
  debug_file << "TIMESTAMP " << " , " <<
                "ERROR " << " , " <<
                "ACCURACY "<< " , " << std::endl;
  debug_file.close();

  std::chrono::steady_clock::time_point time_begin = std::chrono::steady_clock::now();
  for (int i=1; i <= loops_; i++)
  {
    ROS_INFO_STREAM(node_name_ << "-------------------ITERATION " << i << "--------------------------");
    //  Start mock robot and calcs
    ROS_INFO_STREAM(node_name_ << "Start robot ");
    startRobot();

    //  Wait some time to let robot reach max vel.
    ROS_INFO_STREAM(node_name_ << ": Waiting for tobot to reach max vel");
    timedLoopFallbackOnPorts(new std::vector<std::string>{ROBOT_HAS_MAX_VEL_}, 1000);

    //  Stop mock robot
    ROS_INFO_STREAM(node_name_ << ": Robot maintained max vel, stopping it");
    stopRobot();
    double actual_brake_received = pose_interface_.getPortMsg(ROBOT_LOCATION).pose.position.x;
    ROS_DEBUG_STREAM(node_name_ << ": Starting pos stored @ " << ros::Time::now() <<
                    ": " << actual_brake_received);

    //  Wait for mock robot to actually stop
    ROS_INFO_STREAM(node_name_ << ": Waiting for robot to fully stop");
    timedLoopFallbackOnPorts(new std::vector<std::string>{ROBOT_IS_STATIONARY_}, 1000);



    //  Get braking distance ground truth
    double actual_mock_stopped = pose_interface_.getPortMsg(ROBOT_LOCATION).pose.position.x;
    ROS_DEBUG_STREAM(node_name_ << ": Stopping pos stamp @ "
                    <<  pose_interface_.getPortMsg(ROBOT_LOCATION).header.stamp
                    << " stored @ " << ros::Time::now() <<
                    ": " << actual_mock_stopped);
    double actual_braking_distance = actual_brake_received - actual_mock_stopped;

    //  Wait for result & get calculated braking distance
    ros::Duration(0.5).sleep();
    double calc_braking_distance = float_interface_.getPortMsg(CALC_BRAKING_DISTANCE_).data;

    if (actual_braking_distance == 0)
    {
      ROS_WARN_STREAM(node_name_ << ": 0 braking distance, skipping loop");
      continue;
    }

    error = std::abs(std::abs(actual_braking_distance) - std::abs(calc_braking_distance));
    accuracy = 100 - std::abs(error/actual_braking_distance)*100;

    error_list.push_back(error);
    accuracy_list.push_back(accuracy);

    err_avg = ambs_helper::getMean(error_list);
    acc_avg = ambs_helper::getMean(accuracy_list);

    ROS_DEBUG_STREAM(node_name_ << ": Sens dist: " << actual_braking_distance
                    << " Calc dist " << calc_braking_distance);
    if (accuracy <= 90)
      ROS_ERROR_STREAM(node_name_ << ": BAD Error " << error << " BAD Accuracy " << accuracy);
    else
      ROS_WARN_STREAM(node_name_ << ": Error " << error << " Accuracy " << accuracy);
    ROS_WARN_STREAM(node_name_ << ": Error AVG " << err_avg << " Accuracy AVG " << acc_avg);

    debug_file.open(log_folder_path_ + "/benchmarking.csv", std::ios_base::app);
    debug_file << ros::Time::now() << " , " <<
                  error << " , " <<
                  accuracy << " , " << std::endl;
    debug_file.close();

    //  Reset test
    signal_interface_.publishMsgOnPort(RESET_TEST, signal_interface_.constructNewBoolStamped(true));
    float_interface_.resetAllPorts();
    pose_interface_.resetAllPorts();
    signal_interface_.resetAllPorts();

    //  Randomize next run
    ros::Duration(static_cast<float>(std::experimental::randint(5, 15))/10).sleep();
    ROS_INFO_STREAM(" ");
    ROS_INFO_STREAM("-----------------------------------------------------------");
    ROS_INFO_STREAM(" ");
  }
  std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();

  double time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(time_end - time_begin).count();

  double err_std = ambs_helper::getStandardDeviation(error_list);
  double acc_std = ambs_helper::getStandardDeviation(accuracy_list);
  double err_maxdev = ambs_helper::getMaxDeviation(error_list);
  double acc_maxdev = ambs_helper::getMaxDeviation(accuracy_list);

  ROS_WARN_STREAM(node_name_ << ": AVG Accuracy " << acc_avg <<
                   " STD Accuracy " << acc_std <<
                   " MAXDEV Accuracy " << " , " << acc_maxdev);
  ROS_WARN_STREAM(node_name_ << ": AVG Error " << err_avg <<
                   " STD Error " << err_std <<
                   " MAXDEV Error " << " , " << err_maxdev);

  debug_file.open(log_folder_path_ + "/benchmarking.csv", std::ios_base::app);
  debug_file << "AVG Accuracy" << " , " << acc_avg << " , " <<
                "STD Accuracy " << " , " << acc_std << " , " <<
                "MAXDEV Accuracy " << " , " << acc_maxdev << std::endl;
  debug_file << "AVG Error" << " , " << err_avg << " , " <<
                "STD Error " << " , " << err_std << " , " <<
                "MAXDEV Error " << " , " << err_maxdev << std::endl;
  debug_file << "Time elapsed (s)" << " , " << time_elapsed << std::endl;
  debug_file << "Loops" << " , " << loops_ << std::endl;
  debug_file << "Sensor rate" << " , " << sensor_rate_ << std::endl;
  debug_file.close();

  ros::shutdown();
}



/**
 * @brief init function will be called by nodelet, so it is not called anywhere from within class
 *
 * AMBSBaseRunner::startRunner() spawns a timer which executes executeCB()
 */
void BenchBraking::init()
{
  ROS_INFO_STREAM(node_name_ << ": Init class");
  initialiseBaseRunner(nh_, node_name_, extended_inputs_, extended_outputs_);
  pose_interface_.init(pose_inputs_, pose_outputs_, nh_, node_name_);
  float_interface_.init(float_inputs_, float_outputs_, nh_, node_name_);
  debug_logger_.init(nh_, node_name_);

  loops_ = getResolvedParam<int>("loops");
  sensor_rate_ = getResolvedParam<int>("sensor_rate");

  mock_robot_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&BenchBraking::mockRobot, this, _1));
  startRunner();
}




}  // namespace ambs_runners


#endif  // AMBS_COMPONENTS_AMBS_RUNNERS_BENCH_BRAKING_H
