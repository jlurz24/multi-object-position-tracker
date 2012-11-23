#include <ros/ros.h>
#include "pv_filter.h"
#include <message_filters/subscriber.h>
#include <position_tracker/DetectedObjects.h>
#include <position_tracker/DetectedDynamicObjects.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>

using namespace std;

static const double INITIAL_VELOCITY = 0;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;

class DynamicObjectDetector {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    std::vector<PVFilter> pvFilters;
    auto_ptr<message_filters::Subscriber<position_tracker::DetectedObjects> > objectsSub;
    std::string objectName;
    ros::Time lastUpdate;
    ros::Publisher pub;

  public:
    DynamicObjectDetector() : privateHandle("~"){
        privateHandle.getParam("object_name", objectName);
        ROS_INFO("Tracking objects with object name %s", objectName.c_str());
        
        ros::SubscriberStatusCallback cb = boost::bind(&DynamicObjectDetector::connectCB, this);
        pub = nh.advertise<position_tracker::DetectedDynamicObjects>("object_tracks/" + objectName, 1, cb, cb);

	ROS_INFO("Initialization of the dynamic object detector complete");
    }
  private:
    void connectCB(){
      if(pub.getNumSubscribers() == 1){
        startListening();
      }
      else if(pub.getNumSubscribers() == 0) {
        stopListening();
      }
    }

    void stopListening(){
      ROS_INFO("Stopping listeners for multi object detector");
      objectsSub->unsubscribe();
    }

    void startListening(){
      ROS_INFO("Starting to listen for detected object messages");

      if(objectsSub.get() == NULL){
        objectsSub.reset(new message_filters::Subscriber<position_tracker::DetectedObjects>(nh, "/object_locations/" + objectName, 5));
        objectsSub->registerCallback(boost::bind(&DynamicObjectDetector::detectedObjectsCallback, this, _1));
      }
      else {
        objectsSub->subscribe();
      }
      ROS_INFO("Registration for detected objects event complete.");
    }

    void detectedObjectsCallback(const position_tracker::DetectedObjectsConstPtr objects){
      ROS_INFO("Recieved a detected objects callback with %lu objects", objects->positions.size());
      if(objects->positions.size() == 0){
        ROS_INFO("No objects to track");
        return;
      }

      // TODO: Perform frame conversion here.

      // Check if the kalman filters have been initialized.
      if(pvFilters.size() == 0){
        ROS_INFO("Initializing Kalman filters");
        lastUpdate = ros::Time::now();
        pvFilters.resize(objects->positions.size());
        for(unsigned int i = 0; i < objects->positions.size(); ++i){
          vector<double> positions(3);
          positions[0] = objects->positions[i].point.x;
          positions[1] = objects->positions[i].point.y;
          positions[2] = objects->positions[i].point.z;

          vector<double> velocities;
          velocities[0] = INITIAL_VELOCITY;
          velocities[1] = INITIAL_VELOCITY;
          velocities[2] = INITIAL_VELOCITY;

          pvFilters[i].init(positions, velocities);
        }
      }
      else {
        ROS_INFO("Updating Kalman filters");
        ros::Time nextUpdate = ros::Time::now();
        double dt = nextUpdate.toSec() - lastUpdate.toSec();
        lastUpdate = nextUpdate;
        ROS_INFO("DT: %f", dt);

        // Predict
        PointCloudPtr predictedPositions(new PointCloud);
        for(unsigned int i = 0; i < pvFilters.size(); ++i){
          pvFilters[i].setDT(dt);
          PVPair predictedPV = pvFilters[i].predict();
          predictedPositions->points.push_back(pcl::PointXYZ(predictedPV.first[0], predictedPV.first[1], predictedPV.first[2]));
        }

        // Create a point cloud from the detected object centers.
        PointCloudPtr measuredPositions(new PointCloud);
        for(unsigned int i = 0; i < objects->positions.size(); ++i){
          measuredPositions->points.push_back(pcl::PointXYZ(objects->positions[i].point.x, objects->positions[i].point.y, objects->positions[i].point.z));
        }
        // Now associate the predicted points with the measurements.
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputCloud(predictedPositions);
        icp.setInputTarget(measuredPositions);
        pcl::PointCloud<pcl::PointXYZ> final;
        icp.align(final);

        // TODO: Should we do anything if it doesn't converge?
        std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

        // Now update.
        for(unsigned int i = 0; i < pvFilters.size() && i < final.points.size(); ++i){
          vector<double> point(3);
          point[1] = final.points[i].x;
          point[2] = final.points[i].y;
          point[3] = final.points[i].z;
          pvFilters[i].measure(point);
        }
      }

      // Now publish.
      position_tracker::DetectedDynamicObjects trackedObjects;
      for(unsigned int i = 0; i < pvFilters.size(); ++i){
        PVPair x = pvFilters[i].getX();
        geometry_msgs::PointStamped position;
        position.point.x = x.first[0];
        position.point.y = x.first[1];
        position.point.z = x.first[2];

        position.header.frame_id = "/base_footprint";
        position.header.stamp = lastUpdate;
        trackedObjects.positions.push_back(position);
      
        geometry_msgs::TwistStamped velocity;
        velocity.twist.linear.x = x.second[0];
        velocity.twist.linear.y = x.second[1];
        velocity.twist.linear.z = x.second[1];
        velocity.header.frame_id = "/base_footprint";
        velocity.header.stamp = lastUpdate;
        trackedObjects.velocities.push_back(velocity);
      }
      pub.publish(trackedObjects);
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "dynamic_object_detector");
  DynamicObjectDetector dobd;
  ros::spin();
  return 0;
}

