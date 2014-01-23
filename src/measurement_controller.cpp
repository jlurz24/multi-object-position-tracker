#include <ros/ros.h>
#include <position_tracker/StartMeasurement.h>
#include <position_tracker/StopMeasurement.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "fps_measurer");
  ros::NodeHandle nh;

  //! Publishers for starting and stopping measurement.
  if(strcmp(argv[1], "start") == 0){
      ROS_INFO("Starting measurement");
      ros::Publisher startMeasuringPub = nh.advertise<position_tracker::StartMeasurement>("start_measuring", 1,
              true);

      // Notify clients to start measuring.
      position_tracker::StartMeasurement startMeasuringMsg;
      startMeasuringMsg.header.stamp = ros::Time::now();
      startMeasuringPub.publish(startMeasuringMsg);
      ros::spinOnce();
  }
  else if(strcmp(argv[1], "stop") == 0){
      ROS_INFO("Stopping measurement");
      ros::Publisher stopMeasuringPub = nh.advertise<position_tracker::StopMeasurement>("stop_measuring", 1,
              true);
      // Notify clients to stop measuring.
      position_tracker::StopMeasurement stopMeasuringMsg;
      stopMeasuringMsg.header.stamp = ros::Time::now();
      stopMeasuringPub.publish(stopMeasuringMsg);
      ros::spinOnce();
  }
  else {
      ROS_ERROR("Unknown command. Allowed commands are 'start' and 'stop'");
  }
  return 0;
}

