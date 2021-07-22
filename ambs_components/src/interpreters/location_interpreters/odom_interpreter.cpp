#include <string>

#include "ambs_components/ambs_interpreters/location_interpreters/odom_interpreter.h"

namespace ambs_interpreters {

OdomInterpreter::OdomInterpreter()
{
}

OdomInterpreter::OdomInterpreter(ros::NodeHandle nh, std::string node_name):
  nh_(nh),
  node_name_(node_name)
{
}

void OdomInterpreter::init()
{
  ROS_INFO_STREAM(node_name_ << ": Init class");
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(POSE_STAMPED_, queue_size);
  pub_lin_x_ = nh_.advertise<std_msgs::Float64>(TWIST_LINEARX_, queue_size);
  sub_odom_ = nh_.subscribe(ODOM_, queue_size, &OdomInterpreter::odomCB, this);
}

void OdomInterpreter::odomCB(const nav_msgs::OdometryConstPtr msg)
{
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  pub_pose_.publish(pose);

  std_msgs::Float64 lin_x;
  lin_x.data = msg->twist.twist.linear.x;
  pub_lin_x_.publish(lin_x);
}

}  // namespace ambs_interpreters
