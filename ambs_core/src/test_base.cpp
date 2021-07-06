#include <map>
#include <string>
#include "ambs_base/ambs_base.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_base");
  ros::NodeHandle nh;

  std::map<std::string, std::string> control_inputs;
  control_inputs["start"] = "/start";
  control_inputs["stop"] = "/stop";
  control_inputs["reset"] = "/reset";

  std::map<std::string, std::string> control_outputs;
  control_outputs["done"] = "/done";


  ambs_base test_obj = ambs_base(control_inputs, control_outputs, nh);

  ROS_INFO("Hello World");
  ros::spin();
}
