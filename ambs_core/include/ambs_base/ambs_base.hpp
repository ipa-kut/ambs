#ifndef AMBS_BASE_HPP
#define AMBS_BASE_HPP

#include <map>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>

namespace ambs_interfaces {

template <typename T>
struct AMBSPort
{
  AMBSPort() {};
  AMBSPort(std::string topic_name):
          topic_name_(topic_name)
  {}
  std::string topic_name_;
  T msg_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  uint64_t index_;
};

template <typename T>
class AMBSTemplatedInterface
{
public:
  AMBSTemplatedInterface() {};
  ~AMBSTemplatedInterface() {};
  AMBSTemplatedInterface(std::vector<std::string> input_interface,
                         std::vector<std::string> output_interface,
                         ros::NodeHandle nh
                         );

  T getPortMsg(std::string port_name);
  void publishMsgOnPort(std::string port_name, T msg);
  void printPorts();

protected:
  std::deque<boost::mutex> mutexes_;
  std::map<std::string, AMBSPort<T>> interfaces_;
  ros::NodeHandle nh_;
  const unsigned int subscriber_queue_size_ = 10;
  const unsigned int publisher_queue_size_ = 10;

  virtual void templatedCB(const boost::shared_ptr<const T> msg, std::string topic);
  void init(std::vector<std::string> input_interface,
            std::vector<std::string> output_interface);
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------


template<typename T>
AMBSTemplatedInterface<T>::AMBSTemplatedInterface(std::vector<std::string> input_interface,
                                               std::vector<std::string> output_interface,
                                               ros::NodeHandle nh):
  nh_(nh)
{
  mutexes_.resize(input_interface.size());
  init(input_interface, output_interface);
}

template<typename T>
T AMBSTemplatedInterface<T>::getPortMsg(std::string port_name)
{
  if(interfaces_.find(port_name) == interfaces_.end())
  {
    ROS_WARN_STREAM(ros::this_node::getName() << " tried to access non existent port " << port_name);
    T default_msg;
    return default_msg;
  }
  mutexes_[interfaces_[port_name].index_].lock();
  T val = interfaces_[port_name].msg_;
  mutexes_[interfaces_[port_name].index_].unlock();
  return val;
}

template<typename T>
void AMBSTemplatedInterface<T>::publishMsgOnPort(std::string port_name, T msg)
{
  if(interfaces_.find(port_name) == interfaces_.end())
  {
    ROS_WARN_STREAM(ros::this_node::getName() << " tried to access non existent port " << port_name);
    T default_msg;
    return default_msg;
  }
  interfaces_[port_name].pub_.publish(msg);
}

template<typename T>
void AMBSTemplatedInterface<T>::printPorts()
{
   auto iter = interfaces_.begin();
   while(iter != interfaces_.end())
   {
     ROS_INFO_STREAM("Port: " << iter->first << " Indx: " << iter->second.index_
                     << " Sub: " << iter->second.sub_ << " Pub: " << iter->second.pub_);
     ++iter;
   }
   ROS_INFO_STREAM("Total: " << interfaces_.size());
}

template<typename T>
void AMBSTemplatedInterface<T>::templatedCB( boost::shared_ptr<const T> msg, std::string topic)
{
  mutexes_[interfaces_[topic].index_].lock();
  interfaces_[topic].msg_ = *msg.get();
  mutexes_[interfaces_[topic].index_].unlock();
}

template<typename T>
void AMBSTemplatedInterface<T>::init(std::vector<std::string> input_interface,
                                  std::vector<std::string> output_interface)
{
  for(unsigned long i=0; i<input_interface.size(); i++)
  {
    AMBSPort<T> port(input_interface[i]);
    port.index_ = i;
    port.sub_ = nh_.subscribe<T>(port.topic_name_, subscriber_queue_size_,
                              boost::bind(&AMBSTemplatedInterface<T>::templatedCB, this,_1, port.topic_name_)
                              );
    interfaces_[port.topic_name_] = port;
  }

  for(unsigned long i=0; i<output_interface.size(); i++)
  {
    AMBSPort<T> port(output_interface[i]);
    port.index_ = i + input_interface.size();
    port.pub_ = nh_.advertise<T>(port.topic_name_, publisher_queue_size_);
    interfaces_[port.topic_name_] = port;
  }
}


}
#endif // AMBS_BASE_HPP
