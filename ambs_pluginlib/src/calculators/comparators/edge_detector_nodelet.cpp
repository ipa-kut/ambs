#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include "ambs_components/ambs_calculators/comparators/edge_detector.h"

namespace ambs_calculators
{

class EdgeDetectorNodelet : public nodelet::Nodelet
{
public:
  EdgeDetectorNodelet() {}
  ~EdgeDetectorNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        class_pointer_.reset(new EdgeDetector(nh, name));
        class_pointer_->init();
      }

private:
    boost::shared_ptr<EdgeDetector> class_pointer_;
};

}  // namespace ambs_calculators

PLUGINLIB_EXPORT_CLASS(ambs_calculators::EdgeDetectorNodelet,
                       nodelet::Nodelet);
