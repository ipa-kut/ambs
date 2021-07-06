#ifndef AMBS_BASE_HPP
#define AMBS_BASE_HPP

#include <map>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include "ambs_msgs/BoolStamped.h"

namespace ambs_base {

class AmbsBase
{
public:
  AmbsBase(std::map<std::string, std::string> control_input_interface,
           std::map<std::string, std::string> control_output_interface,
           ros::NodeHandle nh
           );
private:
  std::map<std::string, std::string> control_input_interface_;
  std::map<std::string, std::string> control_output_interface_;
  std::vector<ros::Subscriber> subscribers_;
  std::vector<ros::Publisher> publishers_;
  std::deque<boost::mutex> mutexes_;
  std::vector<bool> flagged_variables_;
  ros::NodeHandle nh_;
  unsigned int subscriber_queue_size_ = 10;
  unsigned int publisher_queue_size_ = 10;

  uint64_t getPosOfInputKey(std::string key);
  uint64_t getPosOfOutputKey(std::string key);
  ambs_msgs::BoolStamped getNewBoolStampedMsg(bool data);
  bool getInputFlag(std::string key);
  void pubOutputFlag(std::string key, bool data);
  void callbacksForAllControlInterfaces(const ambs_msgs::BoolStamped::ConstPtr& msg, std::string key);
};

}
#endif // AMBS_BASE_HPP
