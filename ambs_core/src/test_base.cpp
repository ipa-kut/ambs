#include <map>
#include <string>
#include <vector>
#include "ambs_base/ambs_base.hpp"
#include "std_msgs/Float64.h"
#include "ambs_msgs/BoolStamped.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_base");
  ros::NodeHandle nh;

  std::vector<std::string> control_inputs;
  control_inputs.push_back("/start");
  control_inputs.push_back(("/stop"));
  std::vector<std::string> control_outputs;
  control_outputs.push_back("/done");

  std::vector<std::string> float_inputs;
  float_inputs.push_back("/in_float");
  std::vector<std::string> float_outputs;
  float_outputs.push_back("/out_float");


  ambs_interfaces::AMBSTemplatedInterface<ambs_msgs::BoolStamped> bool_interface(control_inputs, control_outputs, nh);
  ambs_interfaces::AMBSTemplatedInterface<std_msgs::Float64> float_interface(float_inputs, float_outputs, nh);

  bool_interface.printPorts();
  float_interface.printPorts();

  ros::Rate loop(2);
  while (ros::ok())
  {
    ROS_INFO_STREAM("Start: " << std::to_string(bool_interface.getPortMsg("/start").data)
                    <<" Stop: " << std::to_string(bool_interface.getPortMsg("/stop").data)
                    << " Done: " << std::to_string(bool_interface.getPortMsg("/done").data));
    ROS_INFO_STREAM("In: " << std::to_string(float_interface.getPortMsg("/in_float").data)
                    <<" Out: " << std::to_string(float_interface.getPortMsg("/out_float").data));
    ROS_INFO("----------------------------------------");
    ROS_INFO(" ");
    loop.sleep();
    ros::spinOnce();
  }
}
