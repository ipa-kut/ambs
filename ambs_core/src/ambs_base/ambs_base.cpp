#include <map>
#include <string>
#include "ambs_base/ambs_base.hpp"

namespace ambs_base
{

AmbsBase::AmbsBase(std::map<std::string, std::string> control_input_interface,
                     std::map<std::string, std::string> control_output_interface,
                     ros::NodeHandle nh):
  control_input_interface_(control_input_interface),
  control_output_interface_(control_output_interface),
  nh_(nh)
{
  mutexes_.resize(control_input_interface_.size());
  subscribers_.resize(control_input_interface_.size());
  publishers_.resize(control_input_interface_.size());
  flagged_variables_.resize(control_input_interface_.size());

  for (auto input : control_input_interface_)
  {
    ROS_INFO_STREAM("Input iface " << input.first << " : "
                    << input.second << " @pos: " << getPosOfInputKey(input.first));
    subscribers_.at(getPosOfInputKey(input.first)) =
        nh_.subscribe<ambs_msgs::BoolStamped>(input.second, subscriber_queue_size_,
                                              boost::bind(
                                                &AmbsBase::callbacksForAllControlInterfaces, this, _1, input.first));
  }

  for (auto output : control_output_interface_)
  {
    ROS_INFO_STREAM("Input iface " << output.first << " : "
                    << output.second << " @pos: " << getPosOfOutputKey(output.first));
    publishers_.at(getPosOfOutputKey(output.first)) =
        nh_.advertise<ambs_msgs::BoolStamped>((output.second), publisher_queue_size_);
  }


  // TESTING ONLY
  ros::Rate rate(100);
  while (ros::ok())
  {
    for (auto input : control_input_interface_)
    {
      ROS_INFO_STREAM("Key: " << input.first << " FlagVal: " << getInputFlag(input.first));
      ros::spinOnce();
    }
    bool result = (getInputFlag("stop") && getInputFlag("start")) || getInputFlag("reset");
    ROS_INFO_STREAM("Result: " << std::to_string(result));
    ROS_INFO("---");
    pubOutputFlag("done", result);
    ROS_INFO("------------");
    ROS_INFO(" ");
    rate.sleep();
  }
  // TESTING ONLY
}

uint64_t AmbsBase::getPosOfInputKey(std::string key)
{
  return static_cast<uint64_t>(std::distance(control_input_interface_.begin(),
                                                  control_input_interface_.find(key)));
}

uint64_t AmbsBase::getPosOfOutputKey(std::string key)
{
  return static_cast<uint64_t>(std::distance(control_output_interface_.begin(),
                                                  control_output_interface_.find(key)));
}

ambs_msgs::BoolStamped AmbsBase::getNewBoolStampedMsg(bool data)
{
  ambs_msgs::BoolStamped msg;
  msg.data = data;
  return msg;
}

bool AmbsBase::getInputFlag(std::string key)
{
  mutexes_.at(getPosOfInputKey(key)).lock();
  bool result = flagged_variables_.at(getPosOfInputKey(key));
  mutexes_.at(getPosOfInputKey(key)).unlock();
  return result;
}

void AmbsBase::pubOutputFlag(std::string key, bool data)
{
  publishers_.at(getPosOfOutputKey(key)).publish(getNewBoolStampedMsg(data));
}

void AmbsBase::callbacksForAllControlInterfaces(const ambs_msgs::BoolStamped::ConstPtr &msg, std::string key)
{
  mutexes_.at(getPosOfInputKey(key)).lock();
  flagged_variables_.at(getPosOfInputKey(key)) = msg->data;
  mutexes_.at(getPosOfInputKey(key)).unlock();
}

}  // namespace ambs_base
