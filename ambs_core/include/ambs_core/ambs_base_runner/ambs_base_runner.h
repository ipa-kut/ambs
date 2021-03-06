#ifndef AMBS_CORE_AMBS_BASE_RUNNER_AMBS_BASE_RUNNER_H
#define AMBS_CORE_AMBS_BASE_RUNNER_AMBS_BASE_RUNNER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "ambs_core/ambs_base_interface/ambs_boolean_interface.hpp"
#include "ambs_msgs/BoolStamped.h"
#include "boost/filesystem.hpp"

namespace ambs_base
{

/**
 * @brief The virtual base class for all runners to inherit from
 * @todo Add parameter container + setters/getters
 */
class AMBSBaseRunner
{
public:
  AMBSBaseRunner() {}
  AMBSBaseRunner(ros::NodeHandle nh, std::string node_name):
    nh_(nh),
    node_name_(node_name)
  {}
  virtual ~AMBSBaseRunner() {}

protected:
  ros::NodeHandle nh_;
  std::string node_name_;
  std::string log_path_;
  std::string log_folder_path_;
  AMBSBooleanInterface signal_interface_;
  const std::string PARAM_LOG_FOLDER_ = "/ambs/log_folder";
  const std::string PARAM_LOG_ROOT_ = "/ambs/log_folder_root";
  const std::string PARAM_ROBOT_NAME_ = "/ambs/robot_name";

  const std::string START_TEST_ = "in_start_test";
  const std::string RESET_TEST_ = "in_reset_test";
  const std::string ROSBAG_BEGAN_ = "in_rosbag_began";
  const std::string START_ROSBAG = "out_start_rosbag";

  const std::string START_ROBOT_ = "out_start_robot";
  const std::string STOP_ROBOT_ = "out_stop_robot";
  const std::string TEST_COMPLETED_ = "out_test_completed";
  const std::string TEST_SUCCEEDED_ = "out_test_succeeded";
  const std::string TEST_FAILED_ = "out_test_failed";
  const std::string DEBUG_TOPIC = "/ambs/debug_messages";

  ambs_msgs::BoolStamped waitForStart();
  ambs_msgs::BoolStamped waitForReset();
  bool initialiseBaseRunner(ros::NodeHandle nh,
                        std::string node_name,
                        std::vector<std::string> extended_bool_inputs,
                        std::vector<std::string> extended_bool_outputs);
  std::string timedLoopFallbackOnPorts(std::vector<std::string> *ports, double time, double rate);
  std::string timedLoopSequenceOnPorts(std::vector<std::string> *ports, double time, double rate);
  template<typename T> T getResolvedParam(std::string param_name);
  virtual void executeCB(const ros::TimerEvent& event) = 0;
  void startRunner();
  void testSucceeded();
  void testFailed();
  void testCompleted();
  void startRobot();
  void stopRobot();

private:
  ros::Timer execute_timer_;
  ros::Subscriber debug_sub_;
  std::vector<std::string> bool_inputs_{START_TEST_, RESET_TEST_, ROSBAG_BEGAN_};
  std::vector<std::string>
  bool_outputs_{START_ROBOT_, STOP_ROBOT_, START_ROSBAG, TEST_COMPLETED_, TEST_SUCCEEDED_, TEST_FAILED_};

  bool createLoggingFolder(std::string path);
  int countSubDirectories(std::string path);
  void debugCallback(const diagnostic_msgs::DiagnosticArrayConstPtr& msg);
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Wait for TRUE on the START port
 * @return The msg that set TRUE
 */
inline ambs_msgs::BoolStamped AMBSBaseRunner::waitForStart()
{
  ROS_INFO_STREAM(node_name_ << ": Wait for Start");
  ambs_msgs::BoolStamped msg = signal_interface_.waitForTrueOnPort(START_TEST_);
  ROS_INFO_STREAM(node_name_ << ": Got Start, waiting for rosbagger to be ready");
  signal_interface_.publishMsgOnPort(START_ROSBAG, signal_interface_.constructNewBoolStamped(true));
  signal_interface_.waitForTrueOnPort(ROSBAG_BEGAN_);
  signal_interface_.publishMsgOnPort(START_ROSBAG, signal_interface_.constructNewBoolStamped(false));
  return msg;
}

/**
 * @brief Wait for TRUE on the RESET port
 * @return The msg that set TRUE
 */
inline ambs_msgs::BoolStamped AMBSBaseRunner::waitForReset()
{
  ROS_INFO_STREAM(node_name_ <<  ": Wait for Reset");
  ambs_msgs::BoolStamped msg = signal_interface_.waitForTrueOnPort(RESET_TEST_);
  ROS_INFO_STREAM(node_name_ << ": Got Reset");
  signal_interface_.resetAllPorts();
  return msg;
}

/**
 * @brief Actual initialisation function
 *
 * startRunner() MUST BE EXPLICITLY called to start the timer
 *
 * @param nh NodeHandle received from the nodelet manager
 */
inline bool AMBSBaseRunner::initialiseBaseRunner(ros::NodeHandle nh,
                                             std::string node_name,
                                             std::vector<std::string> extended_bool_inputs,
                                             std::vector<std::string> extended_bool_outputs)
{
  nh_ = nh;
  node_name_ = node_name;
  bool_inputs_.insert(bool_inputs_.end(), extended_bool_inputs.begin(), extended_bool_inputs.end());
  bool_outputs_.insert(bool_outputs_.end(), extended_bool_outputs.begin(), extended_bool_outputs.end());
  signal_interface_.init(bool_inputs_, bool_outputs_, nh, node_name_);

  std::string log_root;
  std::string robot_name;

  if (!nh_.getParam(PARAM_LOG_ROOT_, log_root))
  {
    ROS_ERROR_STREAM(node_name_ << ": Param " << PARAM_LOG_ROOT_ << " not set!!");
    return false;
  }

  if (!nh_.getParam(PARAM_ROBOT_NAME_, robot_name))
  {
    ROS_ERROR_STREAM(node_name_ << ": Param " << PARAM_ROBOT_NAME_ << " not set!!");
    return false;
  }

  log_path_ = log_root + "/" + robot_name;
  bool result = createLoggingFolder(log_path_);
  if (result)
  {
    std::ofstream debug_file;
    debug_file.open(log_folder_path_ + "/debug.csv", std::ios_base::app);
    debug_file << "TIMESTAMP " << " , " <<
                  "NODE" << " , " <<
                  "LOG LEVEL " << " , " <<
                  "MESSAGE "<< " , " << std::endl;
    debug_file.close();
    debug_sub_ = nh_.subscribe(DEBUG_TOPIC, 1000, &AMBSBaseRunner::debugCallback, this);
  }
  else
  {
    ROS_ERROR_STREAM(node_name_ << ": Folder " << log_path_ << " could not be created!!");
    ros::shutdown();
  }
}

/**
 * @brief Returns the first port in list that has a true message, or if timer expires
 * @param ports The ports to scan sequentially
 * @param time The time to loop
 * @param rate Scan rate. Higher is faster but more taxing. Possible point of optimisation.
 * @return
 */
inline std::string AMBSBaseRunner::timedLoopFallbackOnPorts(std::vector<std::string> *ports,
                                                            double time,
                                                            double rate = 500)
{
  ros::Time start = ros::Time::now();
  ros::Rate loop(rate);
  while (ros::Time::now() - start <= ros::Duration(time))
  {
    for (auto port : *ports)
    {
      if (signal_interface_.isPortValid(port))
      {
        if (signal_interface_.getPortMsg(port).data)
        {
          return port;
        }
      }
      loop.sleep();
    }
  }
  return "";
}

/**
 * @brief Returns the first port in list that has a false message, or if timer expires
 * @param ports The ports to scan sequentially
 * @param time The time to loop
 * @param rate Scan rate. Higher is faster but more taxing. Possible point of optimisation.
 * @return
 */
inline std::string AMBSBaseRunner::timedLoopSequenceOnPorts(std::vector<std::string> *ports,
                                                            double time,
                                                            double rate = 500)
{
  ros::Time start = ros::Time::now();
  ros::Rate loop(rate);
  while (ros::Time::now() - start <= ros::Duration(time))
  {
    for (auto port : *ports)
    {
      if (signal_interface_.isPortValid(port))
      {
        if (!signal_interface_.getPortMsg(port).data)
        {
          return port;
        }
      }
      loop.sleep();
    }
  }
  return "";
}

/**
 * @brief Starts the timer thread. MUST BE EXPLICITLY CALLED.
 */
inline void AMBSBaseRunner::startRunner()
{
  execute_timer_ = nh_.createTimer(ros::Duration(0.01), boost::bind(&AMBSBaseRunner::executeCB, this, _1));
}

/**
 * @brief Publish test results
 */
inline void AMBSBaseRunner::testSucceeded()
{
  signal_interface_.publishMsgOnPort(TEST_FAILED_, signal_interface_.constructNewBoolStamped(false));
  signal_interface_.publishMsgOnPort(TEST_SUCCEEDED_, signal_interface_.constructNewBoolStamped(true));
}

/**
 * @brief Publish test results
 */
inline void AMBSBaseRunner::testFailed()
{
  signal_interface_.publishMsgOnPort(TEST_FAILED_, signal_interface_.constructNewBoolStamped(true));
  signal_interface_.publishMsgOnPort(TEST_SUCCEEDED_, signal_interface_.constructNewBoolStamped(false));
  testCompleted();
}

/**
 * @brief Publish test results
 */
inline void AMBSBaseRunner::testCompleted()
{
  signal_interface_.publishMsgOnPort(TEST_COMPLETED_, signal_interface_.constructNewBoolStamped(true));
}

inline void AMBSBaseRunner::startRobot()
{
  signal_interface_.publishMsgOnPort(STOP_ROBOT_, signal_interface_.constructNewBoolStamped(false));
  signal_interface_.publishMsgOnPort(START_ROBOT_, signal_interface_.constructNewBoolStamped(true));
}

inline void AMBSBaseRunner::stopRobot()
{
  signal_interface_.publishMsgOnPort(START_ROBOT_, signal_interface_.constructNewBoolStamped(false));
  signal_interface_.publishMsgOnPort(STOP_ROBOT_, signal_interface_.constructNewBoolStamped(true));
}

/**
 * @brief Counts the number of subdirectories in a path
 * @param path Path to check
 * @return int The number of directories
 */
inline int AMBSBaseRunner::countSubDirectories(std::string path)
{
  int count = 0;
  boost::filesystem::directory_iterator end_itr;
  for ( boost::filesystem::directory_iterator itr(path);
        itr != end_itr;
        ++itr )
  {
    if (boost::filesystem::is_directory(itr->status()))
    {
      count++;
    }
  }
  return count;
}

/**
 * @brief Callback function for debug topic. Log all messages to file in test folder. Has
 * a globally resolved constant topic name.
 * @param msg The diagnostic message to be logged to file
 */
inline void AMBSBaseRunner::debugCallback(const diagnostic_msgs::DiagnosticArrayConstPtr &msg)
{
  std::ofstream debug_file;
  debug_file.open(log_folder_path_ + "/debug.csv", std::ios_base::app);
  debug_file << msg->header.stamp << " , " <<
                msg->status[0].name << " , " <<
                std::to_string(msg->status[0].level) << " , " <<
                msg->status[0].message << " , " << std::endl;
  debug_file.close();
}

/**
 * @brief Check if param path exists, create it otherwise, then create next logging folder.
 * @param path Root path to create logging folder in
 * @return bool If logging folder was succesfully created
 */
inline bool AMBSBaseRunner::createLoggingFolder(std::string path)
{
  if (!boost::filesystem::exists(path))
  {
    ROS_WARN_STREAM(node_name_ << ": " << path << " does not exist, creating it");
    try
    {
      boost::filesystem::create_directories(path);
    }
    catch (boost::filesystem::filesystem_error err)
    {
      ROS_ERROR_STREAM(node_name_ << ": " << path << " Could not be created : " << err.what());
      return false;
    }
  }

  int count = countSubDirectories(path);
  ROS_DEBUG_STREAM(node_name_ << ": Currently " << std::to_string(count) << " folders, creating next");
  log_folder_path_ = path + "/" + std::to_string(++count);
  boost::filesystem::create_directory(log_folder_path_);
  nh_.setParam(PARAM_LOG_FOLDER_, log_folder_path_);
  ROS_INFO_STREAM(node_name_ << ": Log folder ready: " << log_folder_path_);
  return true;
}

/**
 * @brief Gets a specified parameter by resolving to the ambs/runners namespace
 *
 * @param param_name The parameter to retrieve
 * @tparam T nh.getParam() supports ONLY STRING, INT, DOUBLE, FLOAT and BOOL or vectors thereof
 *
 * example: getResolvedParam<std::vector<double>>("some_param")
 */
template<typename T>
inline T AMBSBaseRunner::getResolvedParam(std::string param_name)
{
  std::string resolved_name = "/ambs/runners/" + node_name_ + "/" + param_name;
  T param_value;
  if (!nh_.getParam(resolved_name, param_value))
  {
    ROS_WARN_STREAM(node_name_ << ": Could not fetch param - " << resolved_name);
  }
  return param_value;
}

}  // namespace ambs_base

#endif  // AMBS_CORE_AMBS_BASE_RUNNER_AMBS_BASE_RUNNER_H
