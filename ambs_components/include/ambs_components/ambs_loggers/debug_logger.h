#ifndef DEBUG_LOGGER_H
#define DEBUG_LOGGER_H

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"

namespace ambs_loggers {

class DebugLogger
{
public:
  DebugLogger() {}

  void init(ros::NodeHandle nh, std::string node_name);
  void logError(std::string message);
  void logInfo(std::string message);
  void logWarn(std::string message);

private:
  ros::NodeHandle nh_;
  std::string node_name_;
  ros::Publisher debug_publisher_;
  const std::string DEBUG_TOPIC = "/ambs/debug_messages";

  diagnostic_msgs::DiagnosticArray constructDiagArrMsg(int8_t level, std::string message);
};

}  // namespace ambs_loggers


#endif // DEBUG_LOGGER_H
