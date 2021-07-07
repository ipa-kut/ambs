#ifndef AMBS_BASE_HPP
#define AMBS_BASE_HPP

#include <map>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include "ambs_msgs/BoolStamped.h"

namespace ambs_base {

template <typename T>
struct AmbsInterface
{
  AmbsInterface() {};
  AmbsInterface(std::string key,
                std::string topic_name):
    key_(key),
    topic_name_(topic_name)
  {}
  std::string key_;
  std::string topic_name_;
  T value_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  uint64_t index_;
};

class AmbsBase
{
public:
  AmbsBase(std::map<std::string, std::string> control_input_interface,
           std::map<std::string, std::string> control_output_interface,
           ros::NodeHandle nh
           );
private:
  std::deque<boost::mutex> mutexes_;
  std::map<std::string, AmbsInterface<bool>> control_interfaces_;
  ros::NodeHandle nh_;
  const unsigned int subscriber_queue_size_ = 10;
  const unsigned int publisher_queue_size_ = 10;

  ambs_msgs::BoolStamped getNewBoolStampedMsg(bool data);
  bool getInputFlag(std::string key);
  void pubOutputFlag(std::string key, bool data);
  void callbacksForAllControlInterfaces(const ambs_msgs::BoolStamped::ConstPtr& msg, std::string key);
};

}
#endif // AMBS_BASE_HPP
