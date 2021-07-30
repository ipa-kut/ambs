#ifndef AMBS_COMPONENTS_AMBS_INTERPRETERS_LOCATION_INTERPRETERS_ODOM_INTERPRETER_H
#define AMBS_COMPONENTS_AMBS_INTERPRETERS_LOCATION_INTERPRETERS_ODOM_INTERPRETER_H

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include "ambs_msgs/BoolStamped.h"
#include "ambs_core/ambs_base_interface/ambs_boolean_interface.hpp"

namespace ambs_interpreters {

/**
 * @brief Used primarily in simulation testing environment instead of focusers with topic_tools/transform to
 * extract Pose of robot and Linear X velocity.
 * Advantage is potential to publish odom data from a nodelet for testing rather than taking from gazebo.
 *
 * Ports:
 * Inputs - Odom
 * Outputs - Pose_stamped, twist_linear_x
 */
class OdomInterpreter
{
public:
  OdomInterpreter();
  OdomInterpreter(ros::NodeHandle nh, std::string node_name);
  void init();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_lin_x_;
  ros::Subscriber sub_odom_;
  std::string node_name_;

  const std::string ODOM_ = "odom";
  const std::string POSE_STAMPED_ = "pose";
  const std::string TWIST_LINEARX_ = "twist_linear_x";

  const unsigned int queue_size = 10;

  void odomCB(const nav_msgs::OdometryConstPtr msg);
};

}  // namespace ambs_interpreters

#endif  // AMBS_COMPONENTS_AMBS_INTERPRETERS_LOCATION_INTERPRETERS_ODOM_INTERPRETER_H
