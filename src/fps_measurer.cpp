#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <position_tracker/DetectedDynamicObjects.h>

using namespace std;

class FPSMeasurer {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    unsigned int iterations;
    ros::Time startTime; 
    message_filters::Subscriber<position_tracker::DetectedDynamicObjects> objectSub;

 public:
    FPSMeasurer() : 
       privateHandle("~"), 
       iterations(0),
       startTime(0),
       objectSub(nh, "object_tracks/balls/positions_velocities", 1){
      objectSub.registerCallback(boost::bind(&FPSMeasurer::callback, this, _1));
      ROS_INFO("Measurement initiated @ %f", ros::Time::now());
      startTime = ros::Time::now();
    }
  
    ~FPSMeasurer(){
      ROS_INFO("Measurement ended");
    }
    
 private:
    void callback(const position_tracker::DetectedDynamicObjectsConstPtr objects){
      ROS_INFO("Received a message @ %f, Start Time @ %f", ros::Time::now().toSec(), startTime.toSec());
   
      iterations++;
      
      double duration = std::max(ros::Time::now().toSec() - startTime.toSec(), 1.0);
      ROS_INFO("Iterations %u, Duration: %f, F/s: %f", iterations, duration, iterations / duration);
   }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "fps_measurer");
  FPSMeasurer fps;
  ros::spin();
  return 0;
}

