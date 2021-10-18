#ifndef AMBS_CORE_AMBS_HELPER_HELPER_H
#define AMBS_CORE_AMBS_HELPER_HELPER_H

#include <algorithm>
#include <vector>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <angles/angles.h>
#include <numeric>

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

/**
 * @brief Get mean of a data series
 * @param data The data series
 * @return The mean
 */
double getMean(std::vector<double> data)
{
  double sum = 0;
  for (double val : data)
  {
    sum += val;
  }
  double mean = sum/data.size();
  return mean;
}

/**
 * @brief Get Standard Deviation of a data series
 * @param data The data series
 * @return The SD
 */
double getStandardDeviation(std::vector<double> data)
{
  double mean = getMean(data);
  double sd = 0;
  for (double val : data)
  {
    sd += pow(val - mean, 2);
  }
  return sqrt(sd/data.size());
}

/**
 * @brief Get Maximum Deviation of a data series
 * @param data The data series
 * @return The MD
 */
double getMaxDeviation(std::vector<double> data)
{
  double mean = getMean(data);
  double md = 0;
  for (double val : data)
  {
    md = std::max(md, std::abs(val - mean));
  }
  return md;
}



}  // namespace ambs_helper

#endif  // AMBS_CORE_AMBS_HELPER_HELPER_H
