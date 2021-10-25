#ifndef AMBS_BASE_HPP
#define AMBS_BASE_HPP

#include <map>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>

namespace ambs_base {

/**
 * @brief Struct that is used to represent one port in the interfaces of components
 *
 * Characterised by topic name, a buffer copy of the latest messgae received on the topic if subscriber,
 * an index to identify its position in the list of all interface ports,
 * as well as a subscriber and a publisher associated with that topic.
 * Currently the ports are initialised ot have a subscriber OR publisher, not both.
 *
 * All interaction with the port (reading values, publishing values) is to be done using this object only
 *
 * @tparam T The type of port
 */


template <typename T>
struct AMBSPort
{
  AMBSPort() {};
  AMBSPort(std::string topic_name):
          topic_name_(topic_name)
  {}
  std::string topic_name_;
  boost::shared_ptr<T> msg_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  uint64_t index_;
};

/**
 * @brief The base templated class used to create interfaces for components
 *
 * Can be used to create any and all types of interfaces using ROS messages as template types.
 * Utilizes AMBSPort to form interfaces
 *
 * Ex: A float interface can be created with 2 input ports and 1 output port (ie topics),
 * with each port represented as an AMBSPorts object.
 *
 * @tparam T The type of interface
 */

template <typename T>
class AMBSTemplatedInterface
{
public:
  AMBSTemplatedInterface() {};
  ~AMBSTemplatedInterface() {};
  AMBSTemplatedInterface(std::vector<std::string> input_interface,
                         std::vector<std::string> output_interface,
                         ros::NodeHandle nh,
                         std::string node_name);

  T getPortMsg(std::string port_name);
  void setPortMsg(std::string port_name, T msg);
  void publishMsgOnPort(std::string port_name, T msg);
  void printPorts();
  bool isPortValid(std::string port_name);
  void init(std::vector<std::string> input_interface,
            std::vector<std::string> output_interface,
            ros::NodeHandle nh,
            std::string node_name);
  void resetPort(std::string port_name);
  void resetAllPorts();

protected:
  std::map<std::string, AMBSPort<T>> interfaces_;
  ros::NodeHandle nh_;
  std::string node_name_;
  const unsigned int subscriber_queue_size_ = 1000;
  const unsigned int publisher_queue_size_ = 1000;

  virtual void templatedCB(const boost::shared_ptr<const T> msg, std::string topic);
};


// ---------------------------------------------------------------------------------------------------------------------
//                                              IMPLEMENTATION
// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Constructor
 *
 * Call init()
 *
 * @param input_interface List of names of topics of ports to serve as subscribers
 * @param output_interface List of name of topics of ports to serve as publishers
 * @param nh Nodehandle passed down from the nodelet manager
 * @param node_name Explicitly necessary becase all nodelets resolve ros::this_node::getName() as nodelet manager
 */
template<typename T> inline
AMBSTemplatedInterface<T>::AMBSTemplatedInterface(std::vector<std::string> input_interface,
                                               std::vector<std::string> output_interface,
                                               ros::NodeHandle nh,
                                               std::string node_name)
{
  init(input_interface, output_interface, nh, node_name);
}

/**
 * @brief Initialisation function, decoupled from constructor for convenience
 *
 * Create subscribers for inputs ports, publishers for output ports,
 * set node handle and save nodelet name
 *
 * @param input_interface List of names of topics of ports to serve as subscribers
 * @param output_interface List of name of topics of ports to serve as publishers
 * @param nh Nodehandle passed down from the nodelet manager
 * @param node_name Explicitly necessary becase all nodelets resolve ros::this_node::getName() as nodelet manager
 */
template<typename T> inline
void AMBSTemplatedInterface<T>::init(std::vector<std::string> input_interface,
                                     std::vector<std::string> output_interface,
                                     ros::NodeHandle nh,
                                     std::string node_name)
{
  node_name_ = node_name;
  nh_ = nh;
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

/**
 * @brief Gets the latest message received from the port, even if it is not a latched topic.
 *
 * @param port_name The name of the topic of the port
 */
template<typename T> inline
T AMBSTemplatedInterface<T>::getPortMsg(std::string port_name)
{
  T default_msg;
  if(isPortValid(port_name))
  {
    return *interfaces_[port_name].msg_;
  }
  else
  {
    /// TODO: Keep this warning approach or not? Should this function be changed?
//    ROS_WARN_STREAM(node_name_ << ": Trying to get msg on port " << port_name << " but it is null!");
    return default_msg;
  }
}

/**
 * @brief Sets the internal message value of a port. DOES NOT PUBLISH.
 *
 * @param port_name The name of the topic of the port
 */
template<typename T> inline
void AMBSTemplatedInterface<T>::setPortMsg(std::string port_name, T msg)
{
  try
  {
    interfaces_[port_name].msg_ = boost::make_shared<T>(msg);
  }
  catch (std::exception e)
  {
    ROS_ERROR_STREAM(node_name_ << ":  setPortMsg exception on port " << port_name << " " << e.what());
  }
}

/**
 * @brief Publish a message on a port.
 *
 * Publishes a shared pointer so as to work with nodelet zero copy transfer
 *
 * @param port_name The name of the topic of the port
 * @param T A ROS msg type
 * @param msg The msg which is automatically cast into a shared pointer.
 */
template<typename T> inline
void AMBSTemplatedInterface<T>::publishMsgOnPort(std::string port_name, T msg)
{
  try
  {
    setPortMsg(port_name,msg);
    interfaces_[port_name].pub_.publish(boost::make_shared<T>(msg));
  }
  catch (std::exception e)
  {
    ROS_ERROR_STREAM(node_name_ << ":  publishMsgOnPort exception on port " << port_name << " " << e.what());
  }
}

/**
 * @brief Prints the member fields of the AMBSPorts object of all ports
 *
 * Used for debugging only
 */
template<typename T> inline
void AMBSTemplatedInterface<T>::printPorts()
{
   auto iter = interfaces_.begin();
   while(iter != interfaces_.end())
   {
     ROS_INFO_STREAM(node_name_ << " Port: " << iter->first << " Indx: " << iter->second.index_
                     << " Sub: " << iter->second.sub_ << " Pub: " << iter->second.pub_
                     << " Val: " << iter->second.msg_);
     ++iter;
   }
   ROS_INFO_STREAM(node_name_ << " Total: " << interfaces_.size());
}

/**
 * @brief Returns if the port has received at least on message
 *
 * @param port_name Port to check
 */
template<typename T>
bool AMBSTemplatedInterface<T>::isPortValid(std::string port_name)
{
  bool result;
  try
  {
    result = interfaces_[port_name].msg_ != nullptr;
  }
  catch (std::exception e)
  {
    ROS_ERROR_STREAM(node_name_ << ":  isPortValid exception on port " << port_name << " " << e.what());
  }
  return result;
}

/**
 * @brief The callback function used by ALL ports
 *
 * Just saves a copy of the received message into the internal buffer in a thread safe manner
 *
 * @param topic The name of the topic of the port
 */
template<typename T> inline
void AMBSTemplatedInterface<T>::templatedCB(const boost::shared_ptr<const T> msg, std::string topic)
{
  try
  {
    interfaces_[topic].msg_ =  boost::make_shared<T>(*msg);
    if ( node_name_.find("edge") <= topic.size() && topic.find("start") <= topic.size())
    {
      ROS_WARN_STREAM(node_name_ << ": Port " << topic << " got " << msg);
    }
  }
  catch (std::exception e)
  {
    ROS_ERROR_STREAM(node_name_ << ":  templatedCB exception " << e.what());
  }
}

/**
 * @brief Reset internal buffer message of a port
 *
 * @param port_name Port to reset
 */
template<typename T> inline
void AMBSTemplatedInterface<T>::resetPort(std::string port_name)
{
  try
  {
    interfaces_[port_name].msg_.reset();
  }
  catch (std::exception e)
  {
    ROS_ERROR_STREAM(node_name_ << ":  resetPort exception on port " << port_name << " " << e.what());
  }
}

/**
 * @brief Reset inernal buffer message of all ports
 */
template<typename T> inline
void AMBSTemplatedInterface<T>::resetAllPorts()
{
  T default_msg;
  auto iter = interfaces_.begin();
  while(iter != interfaces_.end())
  {
    resetPort(iter->first);
    ++iter;
  }
}

}  // namespace ambs_base

#endif // AMBS_BASE_HPP
