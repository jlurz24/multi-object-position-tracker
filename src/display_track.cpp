#include <ros/ros.h>
#include "find_track_impl.h"

int main(int argc, char** argv){
  if(argc < 2){
    ROS_INFO("You must specify the file name on the command line");
    return 0;
  }
  ROS_INFO("Finding track in file: %s", argv[1]);
  position_tracker::showTrackInFile(argv[1]);
}

