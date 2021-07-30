#include <string>

#include <diagnostic_msgs/DiagnosticStatus.h>

#include "ambs_components/ambs_loggers/debug_logger.h"

namespace ambs_loggers {

void DebugLogger::init(ros::NodeHandle nh, std::string node_name)
{
  nh_ = nh;
  node_name_ = node_name;
  debug_publisher_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(DEBUG_TOPIC, 10);
}

void DebugLogger::logError(std::string message)
{
  debug_publisher_.publish(constructDiagArrMsg(diagnostic_msgs::DiagnosticStatus::ERROR, message));
}

void DebugLogger::logInfo(std::string message)
{
  debug_publisher_.publish(constructDiagArrMsg(diagnostic_msgs::DiagnosticStatus::OK, message));
}

void DebugLogger::logWarn(std::string message)
{
  debug_publisher_.publish(constructDiagArrMsg(diagnostic_msgs::DiagnosticStatus::WARN, message));
}

diagnostic_msgs::DiagnosticArray DebugLogger::constructDiagArrMsg(int8_t level, std::string message)
{
  diagnostic_msgs::DiagnosticArray msg;
  diagnostic_msgs::DiagnosticStatus status;
  msg.header.stamp = ros::Time::now();
  status.level = level;
  status.message = message;
  status.name = node_name_;
  msg.status.push_back(status);
  return msg;
}

}  // namespace ambs_loggers
