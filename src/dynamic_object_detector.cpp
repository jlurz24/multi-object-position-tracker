#include <ros/ros.h>
#include "pv_filter.h"
#include <message_filters/subscriber.h>
#include <position_tracker/DetectedObjects.h>
#include <position_tracker/DetectedDynamicObjects.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/lexical_cast.hpp>
#include <pcl/registration/icp_nl.h>
#include <algorithm>
#include <pcl/common/geometry.h>

using namespace std;

#define USE_ICP false

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;
typedef pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> ICP;
typedef boost::shared_ptr<ICP> ICPPtr;

inline Eigen::Vector3f operator-(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
  return Eigen::Vector3f(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}


class DynamicObjectDetector {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    std::vector<boost::shared_ptr<PVFilter> > pvFilters;
    auto_ptr<message_filters::Subscriber<position_tracker::DetectedObjects> > objectsSub;
    
    // Parameters
    std::string objectName;
    std::string frame;
    double initialVelocity;
    double icpMaxCorrespondenceDistance;
    int icpMaxIterations;
    double icpTransformEpsilon;
    double icpEuclideanFitness;
    double icpRansacRejectionThreshold;
    double kalmanObservationNoise;
    double kalmanVelocityNoise;
    double associationEpsilon;
    double associationMaxSuccessScore;

    ros::Time lastUpdate;
    ros::Publisher pub;
    ros::Publisher markerPub;
    ros::Publisher predictedPub;
    ros::Publisher idPub;
    bool subscribed;

  public:
    DynamicObjectDetector() : privateHandle("~"), subscribed(false){
        privateHandle.param<string>("object_name", objectName, "balls");
        privateHandle.param<string>("frame", frame, "/base_footprint");
        privateHandle.param<double>("initial_velocity", initialVelocity, 0.0);
        privateHandle.param<double>("icp_max_correspondence_distance", icpMaxCorrespondenceDistance, 0.05);
        privateHandle.param<int>("icp_max_iterations", icpMaxIterations, 50);
        privateHandle.param<double>("icp_transform_epsilon", icpTransformEpsilon, 1e-6);
        privateHandle.param<double>("icp_euclidian_fitness", icpEuclideanFitness, 1.0);
        privateHandle.param<double>("icp_ransac_rejection_threshold", icpRansacRejectionThreshold, 0.05);

        // Kalman filter parameters
        privateHandle.param<double>("kalman_observation_noise", kalmanObservationNoise, 0.001);
        privateHandle.param<double>("kalman_velocity_noise", kalmanVelocityNoise, 0.1);
        privateHandle.param<double>("association_epsilon", associationEpsilon, 1e-6);
        privateHandle.param<double>("association_max_success_score", associationMaxSuccessScore, 0.5);

        ROS_DEBUG("Tracking objects with object name %s", objectName.c_str());
        
        ros::SubscriberStatusCallback connectCB = boost::bind(&DynamicObjectDetector::startListening, this);
        ros::SubscriberStatusCallback disconnectCB = boost::bind(&DynamicObjectDetector::stopListening, this);

        pub = nh.advertise<position_tracker::DetectedDynamicObjects>("object_tracks/" + objectName + "/positions_velocities", 1, connectCB, disconnectCB);

        markerPub = nh.advertise<visualization_msgs::MarkerArray>("object_tracks/" + objectName + "/markers", 1, connectCB, disconnectCB);

        predictedPub = nh.advertise<visualization_msgs::MarkerArray>("object_tracks/" + objectName + "/predicted_markers", 1, connectCB, disconnectCB);

        idPub = nh.advertise<visualization_msgs::MarkerArray>("object_tracks/" + objectName + "/identities", 1, connectCB, disconnectCB);

	ROS_DEBUG("Initialization of the dynamic object detector complete");
    }

  private:
    void stopListening(){
      if(pub.getNumSubscribers() == 0 && markerPub.getNumSubscribers() == 0 && predictedPub.getNumSubscribers() == 0 && idPub.getNumSubscribers() == 0) {
        ROS_DEBUG("Stopping listeners for multi object detector");
        subscribed = false;
        objectsSub->unsubscribe();
      }
    }

    void startListening(){
      if(subscribed){
        return;
      }

      ROS_DEBUG("Starting to listen for detected object messages");
      subscribed = true;
      if(objectsSub.get() == NULL){
        objectsSub.reset(new message_filters::Subscriber<position_tracker::DetectedObjects>(nh, "/object_locations/" + objectName, 5));
        objectsSub->registerCallback(boost::bind(&DynamicObjectDetector::detectedObjectsCallback, this, _1));
      }
      else {
        objectsSub->subscribe();
      }
      ROS_DEBUG("Registration for detected objects event complete.");
    }

    void detectedObjectsCallback(const position_tracker::DetectedObjectsConstPtr objects){
      ROS_DEBUG("Received detected objects message with %lu objects", objects->positions.size());
      // Check for any data.
      if(objects->positions.size() == 0){
        ROS_DEBUG("No objects to track");
        return;
      }

      // Confirm frames are correct.
      for(unsigned int i = 0; i < objects->positions.size(); ++i){
        if(objects->positions[i].header.frame_id != frame){
          ROS_ERROR("Frame does not match: %s", objects->positions[i].header.frame_id.c_str());
          return;
        }
      }

      // Check if the kalman filters have been initialized.
      // TODO: Move this to the end and make it initialize any unknown points
      //       with filters.
      PointCloudPtr predictedPositions;
      double dt = 1;
      if(objects->positions.size() != pvFilters.size()){
        // TODO: Be smarter when detected number of objects changes.
        //       For now, perform a full reset.

        // Step 0: Initialize the kalman filters with the current positions and initial estimates of velocity.
        ROS_INFO("Initializing Kalman filters");
        lastUpdate = ros::Time::now();
        pvFilters.clear();
        pvFilters.resize(objects->positions.size());
        for(unsigned int i = 0; i < objects->positions.size(); ++i){
          pvFilters[i] = boost::shared_ptr<PVFilter>(new PVFilter(kalmanObservationNoise, kalmanVelocityNoise));
          vector<double> positions(3);
          positions[0] = objects->positions[i].point.x;
          positions[1] = objects->positions[i].point.y;
          positions[2] = objects->positions[i].point.z;

          vector<double> velocities(3);
          velocities[0] = initialVelocity;
          velocities[1] = initialVelocity;
          velocities[2] = 0; // Z velocity is always 0.

          pvFilters[i]->init(positions, velocities);
        }
      }
      else {
        // Perform an update of the filters.

        // Step 1: Calculate delta-t
        ros::Time nextUpdate = ros::Time::now();
        dt = nextUpdate.toSec() - lastUpdate.toSec();

        // Step 2: Predict the new positions
        predictedPositions.reset(new PointCloud);
        for(unsigned int i = 0; i < pvFilters.size(); ++i){
          pvFilters[i]->setDT(dt);
          vector<double> positions;
          vector<double> velocities;
         
          pvFilters[i]->predict(positions);
          predictedPositions->points.push_back(pcl::PointXYZ(positions[0], positions[1], positions[2]));
        }

        // Step 3: Create a point cloud from the detected object centers.
        PointCloudPtr measuredPositions(new PointCloud);
        for(unsigned int i = 0; i < objects->positions.size(); ++i){
          pcl::PointXYZ point(objects->positions[i].point.x, objects->positions[i].point.y, objects->positions[i].point.z);
          measuredPositions->points.push_back(point);
        }

        // Step 4: Associate the predicted points with the measurements.
        const PointCloudConstPtr final = alignClouds(predictedPositions, measuredPositions);
        if(final.get() == NULL){
          return;
        }
        
        // Step 5: Update the filters with the new measurements.
        lastUpdate = nextUpdate;
        for(unsigned int i = 0; i < pvFilters.size() && i < final->points.size(); ++i){
          vector<double> point(3);
          point[0] = final->points[i].x;
          point[1] = final->points[i].y;
          point[2] = final->points[i].z;
          pvFilters[i]->measure(point);
        }
      }      

      // Step 6: Get the current estimates for the state variables
      position_tracker::DetectedDynamicObjectsPtr trackedObjects(new position_tracker::DetectedDynamicObjects);
      for(unsigned int i = 0; i < pvFilters.size(); ++i){
        vector<double> positions;
        vector<double> velocities;
        pvFilters[i]->getX(positions, velocities);
        geometry_msgs::PointStamped position;
        position.point.x = positions[0];
        position.point.y = positions[1];
        position.point.z = positions[2];

        position.header.frame_id = frame;
        position.header.stamp = lastUpdate;
        trackedObjects->positions.push_back(position);
      
        geometry_msgs::TwistStamped velocity;
        velocity.twist.linear.x = velocities[0];
        velocity.twist.linear.y = velocities[1];
        velocity.twist.linear.z = velocities[2];
        velocity.header.frame_id = frame;
        velocity.header.stamp = lastUpdate;
        trackedObjects->velocities.push_back(velocity);
      }

      // Step 8: Publish the results
      if(predictedPub.getNumSubscribers() > 0 && predictedPositions != NULL){
        publishPredictedPositions(predictedPositions, lastUpdate);
      }

      pub.publish(trackedObjects);
 
      if(markerPub.getNumSubscribers() > 0){
        publishPVArrows(trackedObjects);
      }

      if(idPub.getNumSubscribers() > 0){
        publishIDs(trackedObjects);
      }
    }

#if USE_ICP
   ICPPtr createICP(const PointCloudConstPtr inputCloud, const PointCloudConstPtr targetCloud) const {
     ICPPtr icp(new ICP);
     icp->setInputCloud(inputCloud);
     icp->setInputTarget(targetCloud);

     icp->setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance);
     icp->setMaximumIterations(icpMaxIterations);
     icp->setTransformationEpsilon(icpTransformEpsilon);
     icp->setEuclideanFitnessEpsilon(icpEuclideanFitness);
     icp->setRANSACOutlierRejectionThreshold(icpRansacRejectionThreshold);
     return icp;
   }

   PointCloudConstPtr alignClouds(const PointCloudConstPtr predictedPositions, const PointCloudConstPtr measuredPositions) const {
     ICPPtr icpPredicted = createICP(measuredPositions, predictedPositions);
     PointCloudPtr finalPredicted(new PointCloud);
     icpPredicted->align(*finalPredicted);
     
     if(!icpPredicted->hasConverged()){
       ROS_DEBUG("Predicted ICP did not converge. Score: %f", icpPredicted->getFitnessScore());
     }

     // Try again with the current positions.
     ICPPtr icpCurrent = createICP(measuredPositions, currentPositions);
     PointCloudPtr finalCurrent(new PointCloud);
     icpCurrent->align(*finalCurrent);
     if(!icpCurrent->hasConverged()){
       ROS_DEBUG("Current ICP did not converge. Score: %f", icpCurrent->getFitnessScore());
     }
     
     ROS_DEBUG("Fitness score with current positions %f and predicted positions %f", icpCurrent->getFitnessScore(), icpPredicted->getFitnessScore());

     if(!icpPredicted->hasConverged() && !icpCurrent->hasConverged()){
       ROS_DEBUG("Neither predicted nor current ICPs converged");
       return PointCloudPtr();
     }
     return icpPredicted->getFitnessScore() < icpCurrent->getFitnessScore() ? finalPredicted : finalCurrent;
   }
#else
  
   PointCloudConstPtr alignClouds(const PointCloudConstPtr predictedPositions, const PointCloudConstPtr measuredPositions) const {

   ROS_DEBUG("Aligning %lu points", predictedPositions->points.size());

   // Create a vector that will represent the ordering. Start with the lowest lexographic ordering.
  vector<unsigned int> currentOrder;
  for(unsigned int i = 0; i < measuredPositions->points.size(); ++i){
    currentOrder.push_back(i);
  }

  // Set the initial order as the best.
  vector<unsigned int> closestOrder = currentOrder;
  double lowestDistance = numeric_limits<double>::max();

  // Now search all permutations.
  while(next_permutation(currentOrder.begin(), currentOrder.end())){
    // Sum the distances.
    double distanceSum = 0;
    for(unsigned int i = 0; i < currentOrder.size(); ++i){
      distanceSum += pcl::geometry::squaredDistance(predictedPositions->points[i], measuredPositions->points[currentOrder[i]]);
    }

    if(distanceSum < lowestDistance){
      lowestDistance = distanceSum;
      closestOrder = currentOrder;
      if(lowestDistance < associationEpsilon){
        ROS_DEBUG("Existing early from association search");
      }
    }
  }

  ROS_DEBUG("Lowest total distance was %f", lowestDistance);

  // Excercised all perumutations.
  PointCloudPtr alignedPositions(new PointCloud);
  if(lowestDistance <= associationMaxSuccessScore){
    for(unsigned int i = 0; i < measuredPositions->points.size(); ++i){
      alignedPositions->points.push_back(measuredPositions->points[closestOrder[i]]);
    }
  }
  else {
    ROS_INFO("Failed to associate measurements. Score was %f.", lowestDistance);
  }
  return alignedPositions;
}
#endif

   void publishPVArrows(const position_tracker::DetectedDynamicObjectsConstPtr objects) const {

     double DT_FOR_VIZ = 2;

     visualization_msgs::MarkerArrayPtr markers(new visualization_msgs::MarkerArray);
     for(unsigned int i = 0; i < objects->positions.size(); ++i){
        visualization_msgs::Marker marker;
        marker.id = i;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.header.stamp = lastUpdate;
        marker.header.frame_id = frame;
        marker.points.resize(2);
        marker.points[0] = objects->positions[i].point;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.04;
        marker.color.a = 1;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 0;

        // Calculate the next point
        marker.points[1] = objects->positions[i].point;
        marker.points[1].x += objects->velocities[i].twist.linear.x * DT_FOR_VIZ;
        marker.points[1].y += objects->velocities[i].twist.linear.y * DT_FOR_VIZ;
        marker.points[1].z += objects->velocities[i].twist.linear.z * DT_FOR_VIZ;
        markers->markers.push_back(marker);
     }
     markerPub.publish(markers);
   }

   void publishIDs(const position_tracker::DetectedDynamicObjectsConstPtr objects) const {
     visualization_msgs::MarkerArrayPtr markers(new visualization_msgs::MarkerArray);
     for(unsigned int i = 0; i < objects->positions.size(); ++i){
        visualization_msgs::Marker marker;
        marker.id = i;
        marker.text = boost::lexical_cast<string>(i + 1);
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.header.stamp = lastUpdate;
        marker.header.frame_id = frame;
        marker.color.a = 1;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
        marker.pose.position = objects->positions[i].point;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        markers->markers.push_back(marker);
     }
     idPub.publish(markers);
   }

   void publishPredictedPositions(const PointCloudConstPtr predictedPositions, ros::Time stamp) const {
     visualization_msgs::MarkerArrayPtr predictedMarkers(new visualization_msgs::MarkerArray);
     for(unsigned int i = 0; i < predictedPositions->points.size(); ++i){
       visualization_msgs::Marker marker;
       marker.id = i;
       marker.action = visualization_msgs::Marker::ADD;
       marker.type = visualization_msgs::Marker::SPHERE;
       marker.header.stamp = stamp;
       marker.header.frame_id = frame;
       marker.pose.position.x = predictedPositions->points[i].x;
       marker.pose.position.y = predictedPositions->points[i].y;
       marker.pose.position.z = predictedPositions->points[i].z;
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
     predictedPub.publish(predictedMarkers);
   }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "dynamic_object_detector");
  DynamicObjectDetector dobd;
  ros::spin();
  return 0;
}

