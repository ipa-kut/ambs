#ifndef AMBS_COMPONENTS_AMBS_LOGGERS_DEBUG_LOGGER_H
#define AMBS_COMPONENTS_AMBS_LOGGERS_DEBUG_LOGGER_H

#include <string>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "ambs_core/ambs_base_interface/ambs_base_interface.hpp"

namespace ambs_loggers {

/**
 * @brief Components may use objects of this class to send debug logging messages to the runner.
 */
class DebugLogger
{
public:
  DebugLogger() {}

  void init(ros::NodeHandle nh, std::string node_name);
  /**
   * @brief Log message with level 2
   * @param message Message to log
   */
  void logError(std::string message);
  /**
   * @brief Log message with level 0
   * @param message Message to log
   */
  void logInfo(std::string message);
  /**
   * @brief Log message with level 1
   * @param message Message to log
   */
  void logWarn(std::string message);

private:
  ros::NodeHandle nh_;
  std::string node_name_;
  ros::Publisher debug_publisher_;
  const std::string DEBUG_TOPIC = "/ambs/debug_messages";

  diagnostic_msgs::DiagnosticArray constructDiagArrMsg(int8_t level, std::string message);
};

}  // namespace ambs_loggers


#endif  // AMBS_COMPONENTS_AMBS_LOGGERS_DEBUG_LOGGER_H
