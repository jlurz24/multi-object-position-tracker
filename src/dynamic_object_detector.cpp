#include <ros/ros.h>
#include "pv_filter.h"
#include <message_filters/subscriber.h>
#include <position_tracker/DetectedObjects.h>
#include <position_tracker/DetectedDynamicObjects.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

static const double INITIAL_VELOCITY = 0;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;

class DynamicObjectDetector {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    std::vector<boost::shared_ptr<PVFilter> > pvFilters;
    auto_ptr<message_filters::Subscriber<position_tracker::DetectedObjects> > objectsSub;
    std::string objectName;
    ros::Time lastUpdate;
    ros::Publisher pub;
    ros::Publisher markerPub;
    ros::Publisher predictedPub;

  public:
    DynamicObjectDetector() : privateHandle("~"){
        privateHandle.param<string>("object_name", objectName, "balls");
        ROS_INFO("Tracking objects with object name %s", objectName.c_str());
        
        ros::SubscriberStatusCallback connectCB = boost::bind(&DynamicObjectDetector::startListening, this);
        ros::SubscriberStatusCallback disconnectCB = boost::bind(&DynamicObjectDetector::stopListening, this);

        pub = nh.advertise<position_tracker::DetectedDynamicObjects>("object_tracks/" + objectName, 1, connectCB, disconnectCB);

        markerPub = nh.advertise<visualization_msgs::MarkerArray>("object_tracks/markers", 1, connectCB, disconnectCB);

        predictedPub = nh.advertise<visualization_msgs::MarkerArray>("object_tracks/predicted_markers", 1, connectCB, disconnectCB);

	ROS_INFO("Initialization of the dynamic object detector complete");
    }
  private:
    void stopListening(){
      if(pub.getNumSubscribers() == 0 && markerPub.getNumSubscribers() == 0 && predictedPub.getNumSubscribers() == 0) {
        ROS_INFO("Stopping listeners for multi object detector");
        objectsSub->unsubscribe();
      }
    }

    void startListening(){
      if(pub.getNumSubscribers() + markerPub.getNumSubscribers() + predictedPub.getNumSubscribers() != 1){
        return;
      }

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
          pvFilters[i] = boost::shared_ptr<PVFilter>(new PVFilter);
          vector<double> positions(3);
          positions[0] = objects->positions[i].point.x;
          positions[1] = objects->positions[i].point.y;
          positions[2] = objects->positions[i].point.z;

          vector<double> velocities(3);
          velocities[0] = INITIAL_VELOCITY;
          velocities[1] = INITIAL_VELOCITY;
          velocities[2] = INITIAL_VELOCITY;

          pvFilters[i]->init(positions, velocities);
        }
        return;
      }
  
      // Update
      ros::Time nextUpdate = ros::Time::now();
      double dt = nextUpdate.toSec() - lastUpdate.toSec();
      lastUpdate = nextUpdate;

      // Predict
      PointCloudPtr predictedPositions(new PointCloud);
      visualization_msgs::MarkerArrayPtr predictedMarkers(new visualization_msgs::MarkerArray);
        
      for(unsigned int i = 0; i < pvFilters.size(); ++i){
        pvFilters[i]->setDT(dt);
        vector<double> positions;
        pvFilters[i]->predict(positions);
        predictedPositions->points.push_back(pcl::PointXYZ(positions[0], positions[1], positions[2]));

        // TODO: Move to separate function
        if(predictedPub.getNumSubscribers() > 0){
            visualization_msgs::Marker marker;
            marker.id = i;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.header.stamp = lastUpdate;
            marker.header.frame_id = "/base_footprint";
            marker.pose.position.x = positions[0];
            marker.pose.position.y = positions[1];
            marker.pose.position.z = positions[2];
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.04;
            marker.color.a = 1;
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
            predictedMarkers->markers.push_back(marker);
        }
      }

      if(predictedPub.getNumSubscribers() > 0){
        predictedPub.publish(predictedMarkers);
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

      // Set the max correspondence distance to 25cm (e.g., correspondences with higher distances will be ignored)
      icp.setMaxCorrespondenceDistance(0.25);
      // Set the maximum number of iterations (criterion 1)
      icp.setMaximumIterations(50);
      // Set the transformation epsilon (criterion 2)
      icp.setTransformationEpsilon (1e-8);
      // Set the euclidean distance difference epsilon (criterion 3)
      icp.setEuclideanFitnessEpsilon(1);
      pcl::PointCloud<pcl::PointXYZ> final;
      icp.align(final);

      // TODO: What else should we do to recover.
      if(!icp.hasConverged()){
        ROS_INFO("ICP did not converge. Score: %f", icp.getFitnessScore());
        return;
      }
        
      // Now update.
      for(unsigned int i = 0; i < pvFilters.size() && i < final.points.size(); ++i){
        vector<double> point(3);
        point[0] = final.points[i].x;
        point[1] = final.points[i].y;
        point[2] = final.points[i].z;
        pvFilters[i]->measure(point);
      }
      

      // Now publish.
      visualization_msgs::MarkerArrayPtr markers(new visualization_msgs::MarkerArray);
      position_tracker::DetectedDynamicObjectsPtr trackedObjects(new position_tracker::DetectedDynamicObjects);

      for(unsigned int i = 0; i < pvFilters.size(); ++i){
        vector<double> positions;
        vector<double> velocities;
        pvFilters[i]->getX(positions, velocities);
        geometry_msgs::PointStamped position;
        position.point.x = positions[0];
        position.point.y = positions[1];
        position.point.z = positions[2];

        position.header.frame_id = "/base_footprint";
        position.header.stamp = lastUpdate;
        trackedObjects->positions.push_back(position);
      
        geometry_msgs::TwistStamped velocity;
        velocity.twist.linear.x = velocities[0];
        velocity.twist.linear.y = velocities[1];
        velocity.twist.linear.z = velocities[2];
        velocity.header.frame_id = "/base_footprint";
        velocity.header.stamp = lastUpdate;
        trackedObjects->velocities.push_back(velocity);

        // TODO: Move to separate function
        if(markerPub.getNumSubscribers() > 0){
          visualization_msgs::Marker marker;
          marker.id = i;
          marker.action = visualization_msgs::Marker::ADD;
          marker.type = visualization_msgs::Marker::ARROW;
          marker.header = position.header;
          marker.points.resize(2);
          marker.points[0] = position.point;
          marker.scale.x = marker.scale.y = marker.scale.z = 0.04;
          marker.color.a = 1;
          marker.color.r = 0;
          marker.color.g = 1;
          marker.color.b = 0;

          // Calculate the next point
          double dt = 1;
          marker.points[1] = position.point;
          ROS_INFO("Velocities %f %f %f", velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z);
          marker.points[1].x += velocity.twist.linear.x * dt;
          marker.points[1].y += velocity.twist.linear.y * dt;
          marker.points[1].z += velocity.twist.linear.z * dt;
          markers->markers.push_back(marker);
        }
      }
      pub.publish(trackedObjects);
 
      if(markerPub.getNumSubscribers() > 0){
        markerPub.publish(markers);
      }
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "dynamic_object_detector");
  DynamicObjectDetector dobd;
  ros::spin();
  return 0;
}

