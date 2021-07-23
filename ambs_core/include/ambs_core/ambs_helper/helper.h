#ifndef AMBS_HELPER_HELPER_H
#define AMBS_HELPER_HELPER_H

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <angles/angles.h>

namespace ambs_helper
{

double getYawDegreesFromQuaternion(geometry_msgs::Quaternion orientation)
{
  tf::Quaternion quat;
  quat.normalize();
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(orientation, quat);
  tf::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch, yaw);
  return angles::to_degrees(yaw);
}

double getTranslationDiffFromPoses(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped stop_pose)
{
  double delta_x = std::abs(start_pose.pose.position.x - stop_pose.pose.position.x);
  double delta_y = std::abs(start_pose.pose.position.y - stop_pose.pose.position.y);
  return std::sqrt((delta_x*delta_x) + (delta_y*delta_y));
}

double getYawDiffFromPoses(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped stop_pose)
{
  return std::abs(getYawDegreesFromQuaternion(start_pose.pose.orientation) -
        getYawDegreesFromQuaternion(stop_pose.pose.orientation));
}


}  // namespace ambs_helper

#endif  // AMBS_HELPER_HELPER_H
