#include <ros/ros.h>
#include "pv_filter.h"

using namespace std;

class DynamicObjectDetector {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    std::vector<PVFilter> pvFilters;    
 public:
    DynamicObjectDetector() : privateHandle("~"){
	ROS_INFO("Initialization of the dynamic object detector complete");
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "dynamic_object_detector");
  DynamicObjectDetector dobd;
  ros::spin();
  return 0;
}

