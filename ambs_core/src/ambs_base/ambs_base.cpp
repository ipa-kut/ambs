#include <map>
#include <string>
#include "ambs_base/ambs_base.hpp"

namespace ambs_base
{

AmbsBase::AmbsBase(std::map<std::string, std::string> control_input_interface,
                     std::map<std::string, std::string> control_output_interface,
                     ros::NodeHandle nh):
  nh_(nh)
{
  mutexes_.resize(control_input_interface.size());

  for (auto input : control_input_interface)
  {
    AmbsInterface<bool> interface(input.first, input.second);
    interface.index_ = static_cast<uint64_t>(std::distance(control_input_interface.begin(),
                                                           control_input_interface.find(input.first)));
    interface.sub_ =
        nh_.subscribe<ambs_msgs::BoolStamped>(input.second, subscriber_queue_size_,
                                              boost::bind(
                                                &AmbsBase::callbacksForAllControlInterfaces, this, _1, input.first));
        control_interfaces_[input.first] = interface;
        ROS_INFO_STREAM("Input iface " << input.first << " : "
                        << input.second << " @pos: " << interface.index_);
  }

  for (auto output : control_output_interface)
  {
    AmbsInterface<bool> interface(output.first, output.second);
    interface.index_ = static_cast<uint64_t>(std::distance(control_output_interface.begin(),
                                                           control_output_interface.find(output.first)));
    interface.pub_ =
        nh_.advertise<ambs_msgs::BoolStamped>((output.second), publisher_queue_size_);
    control_interfaces_[output.first] = interface;
    ROS_INFO_STREAM("Output iface " << output.first << " : "
                    << output.second << " @pos: " << interface.index_);
  }

  // TESTING ONLY
  ros::Rate rate(100);
  while (ros::ok())
  {
    for (auto input : control_input_interface)
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

ambs_msgs::BoolStamped AmbsBase::getNewBoolStampedMsg(bool data)
{
  ambs_msgs::BoolStamped msg;
  msg.data = data;
  return msg;
}

bool AmbsBase::getInputFlag(std::string key)
{
  mutexes_.at(control_interfaces_[key].index_).lock();
  bool result = control_interfaces_[key].value_;
  mutexes_.at(control_interfaces_[key].index_).unlock();
  return result;
}

void AmbsBase::pubOutputFlag(std::string key, bool data)
{
  control_interfaces_[key].pub_.publish(getNewBoolStampedMsg(data));
}

void AmbsBase::callbacksForAllControlInterfaces(const ambs_msgs::BoolStamped::ConstPtr &msg, std::string key)
{
  mutexes_.at(control_interfaces_[key].index_).lock();
  control_interfaces_[key].value_ = msg->data;
  mutexes_.at(control_interfaces_[key].index_).unlock();
}

}  // namespace ambs_base
