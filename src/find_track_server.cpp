#include <ros/ros.h>
#include <position_tracker/FindTrack.h>
#include "find_track_impl.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

using namespace cv;
using namespace position_tracker;

class FindTrackServer {
  public:
    FindTrackServer(): privateNh("~"){
      privateNh.getParam("debug", debug);
      privateNh.param<int>("max-attempts", maxAttempts, 5);
      privateNh.param<int>("image-aquire-timeout", imageAquireTimeout, 30);
      privateNh.param<string>("image-topic", imageTopic, "/wide_stereo/left/image_rect_color");
      privateNh.param<string>("points-topic", pointsTopic, "/wide_stereo/left/points");
      
      service = nh.advertiseService("find_track", &FindTrackServer::findTrack, this);
    }

    bool findTrack(FindTrack::Request& req, FindTrack::Response& res){
      bool foundTrack = false;
      int attempts = 0; 

      sensor_msgs::PointCloud2ConstPtr imagePoints;
      std::vector<cv::Point> track;
      sensor_msgs::ImageConstPtr image;

      while(!foundTrack && attempts++ < maxAttempts){   
        ROS_INFO("Waiting for image message");
   
        // Wait 30 seconds for a color image
        image = ros::topic::waitForMessage<sensor_msgs::Image>(imageTopic, nh, ros::Duration(imageAquireTimeout));
  
        ROS_INFO("Received an image with width %u height %u", image->width, image->height);

        // Wait for a 3d points message;
        ROS_INFO("Waiting for 3d points message");
        imagePoints = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pointsTopic);

        // Begin processing the image.
        cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(image);

       // Save the image for debugging.
       if(debug){
         std::string debugFileName = "debug_image.jpg";
         privateNh.getParam("debug_file_name", debugFileName);
         ROS_INFO("Saving image to %s", debugFileName.c_str());
         cv::imwrite(debugFileName, cvImg->image);
       }
     
       ROS_INFO("Finding track");
       track = position_tracker::findTrack(cvImg->image);
   
       if(track.size() == 0){
         ROS_INFO("Track not found. Retrying with a fresh image");
         ros::Duration(1.0).sleep();
       }
       else {
         foundTrack = true;
       }
     }

     if(!foundTrack){
       ROS_INFO("Could not find the track");
       return false; // TODO: Set error message
     }

     // Find the points in the 3d point plane.
     pcl::PointCloud<pcl::PointXYZ> depthCloud;
     pcl::fromROSMsg(*imagePoints, depthCloud);

     std::vector<pcl::PointXYZ> contour3d;
     contour3d.resize(track.size());
     for(unsigned int i = 0; i < contour3d.size(); ++i){
       contour3d[i] = depthCloud.points.at(track[i].y * image->width + track[i].x);
     }

     // Now convert from image from camera frame to world
     ros::Time now = ros::Time::now();
     tf.waitForTransform("/base_footprint", image->header.frame_id, now, ros::Duration(10.0));
   
     for(unsigned int i = 0; i < contour3d.size(); ++i){   
       geometry_msgs::PointStamped resultPoint;
       resultPoint.header.frame_id = "/base_footprint";
       geometry_msgs::PointStamped imagePoint;
       imagePoint.header.frame_id = image->header.frame_id;
       imagePoint.header.stamp = now;
       imagePoint.point.x = contour3d[i].x;
       imagePoint.point.y = contour3d[i].y;
       imagePoint.point.z = contour3d[i].z;
       tf.transformPoint("/base_footprint", imagePoint, resultPoint);
       res.track.push_back(resultPoint);
     }

     return true;
  }
 
  private:
     ros::NodeHandle nh;
     ros::NodeHandle privateNh;
     ros::ServiceServer service;
     tf::TransformListener tf;
     int imageAquireTimeout;
     int maxAttempts;
     string imageTopic;
     string pointsTopic;
     bool debug;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "find_track_server");
  FindTrackServer findTrackServer;
  ros::spin();
  return 0;
}
