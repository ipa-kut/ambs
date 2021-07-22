#ifndef AMBS_HELPER_HELPER_H
#define AMBS_HELPER_HELPER_H

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <angles/angles.h>

namespace ambs_base
{

double getYawDegreesFromQuaternion(geometry_msgs::Quaternion orientation)
{
  tf::Quaternion quat;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(orientation, quat);
  quat.normalize();
  tf::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch, yaw);
  return angles::to_degrees(yaw);
}


}  // namespace ambs_base

#endif  // AMBS_HELPER_HELPER_H
