#include <ros/ros.h>
#include "pv_filter.h"
#include <message_filters/subscriber.h>
#include <position_tracker/DetectedObjects.h>
#include <position_tracker/DetectedDynamicObjects.h>
#include <pcl16/point_types.h>
#include <pcl16_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <pcl16/common/geometry.h>

using namespace std;
using namespace pcl16;

typedef PointCloud<PointXYZ> PointCloudXYZ;
typedef PointCloudXYZ::ConstPtr PointCloudXYZConstPtr;
typedef PointCloudXYZ::Ptr PointCloudXYZPtr;
typedef vector<boost::shared_ptr<PVFilter> > PVFilterVector;

inline Eigen::Vector3f operator-(const PointXYZ& p1, const PointXYZ& p2) {
  return Eigen::Vector3f(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

inline bool operator==(const PointXYZ& p1, const PointXYZ& p2){
  return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
}

inline bool operator!=(const PointXYZ& p1, const PointXYZ&p2){
  return !(p1 == p2);
}

class DynamicObjectDetector {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    PVFilterVector pvFilters;
    auto_ptr<message_filters::Subscriber<position_tracker::DetectedObjects> > objectsSub;

    // Parameters
    std::string objectName;
    std::string frame;
    double initialVelocity;
    double kalmanObservationNoise;
    double kalmanAccelerationDist;
    double associationEpsilon;
    double associationMaxSuccessScore;
    double filterStaleThreshold;
    double maxCorrelationDistance;

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

        // Kalman filter parameters
        privateHandle.param<double>("kalman_observation_noise", kalmanObservationNoise, 0.001);
        privateHandle.param<double>("kalman_acceleration_dist", kalmanAccelerationDist, 0.1);
        privateHandle.param<double>("association_epsilon", associationEpsilon, 1e-6);
        privateHandle.param<double>("association_max_success_score", associationMaxSuccessScore, 0.5);
        privateHandle.param<double>("filter_stale_threshold", filterStaleThreshold, 15);
        privateHandle.param<double>("max_correlation_distance", maxCorrelationDistance, 5.0);

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
      ROS_DEBUG("Currently %lu known objects", pvFilters.size());

      if(objects->positions.size() > pvFilters.size()){
        ROS_INFO("Received %lu measurements and only have %lu filters", objects->positions.size(), pvFilters.size());
      }

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

      // Step 1: Predict the new positions
      ros::Time measurementTime = objects->positions[0].header.stamp;
      PointCloudXYZPtr predictedPositions(new PointCloudXYZ);
      for(unsigned int i = 0; i < pvFilters.size(); ++i){
        vector<double> positions;
        vector<double> velocities;

        pvFilters[i]->predict(positions, measurementTime);
        predictedPositions->points.push_back(PointXYZ(positions[0], positions[1], positions[2]));
      }

      // Step 2: Create a point cloud from the detected object centers.
      PointCloudXYZPtr measuredPositions(new PointCloudXYZ);
      for(unsigned int i = 0; i < objects->positions.size(); ++i){
        PointXYZ point(objects->positions[i].point.x, objects->positions[i].point.y, objects->positions[i].point.z);
        measuredPositions->points.push_back(point);
      }

      // Step 3: Associate the predicted points with the measurements.
      const PointCloudXYZPtr unalignedMeasurements(new PointCloudXYZ);
      const PointCloudXYZConstPtr final = alignClouds(predictedPositions, measuredPositions, unalignedMeasurements);

      // Step 4: Update the associated filters with the new measurements.
      for(unsigned int i = 0; i < final->points.size(); ++i){
        // Ignore null measurements.
        if(final->points[i].x != numeric_limits<double>::infinity()){
          vector<double> point(3);
          point[0] = final->points[i].x;
          point[1] = final->points[i].y;
          point[2] = final->points[i].z;
          pvFilters[i]->measure(point, measurementTime);
        }
        else {
          ROS_DEBUG("Ignoring a null measurement");
        }
      }

      // Step 4a: Prune any filters that have not been updated
      //          recently.
      for(PVFilterVector::iterator i = pvFilters.begin(); i != pvFilters.end(); ++i){
        ROS_DEBUG("Last update time for filter: %f, measurement time: %f, threshold: %f", (*i)->getLastUpdate().toSec(), measurementTime.toSec(), filterStaleThreshold);
        if(measurementTime.toSec() - (*i)->getLastUpdate().toSec() > filterStaleThreshold){
           ROS_INFO("Pruning a filter that has not been updated since %f", (*i)->getLastUpdate().toSec());
           pvFilters.erase(i);
         }
      }

      // Step 5: Get the current estimates for the state variables
      position_tracker::DetectedDynamicObjectsPtr trackedObjects(new position_tracker::DetectedDynamicObjects);
      trackedObjects->header.stamp = measurementTime;

      for(unsigned int i = 0; i < pvFilters.size(); ++i){
        vector<double> positions;
        vector<double> velocities;
        pvFilters[i]->getX(positions, velocities);
        geometry_msgs::PointStamped position;
        position.point.x = positions[0];
        position.point.y = positions[1];
        position.point.z = positions[2];

        position.header.frame_id = frame;
        position.header.stamp = measurementTime;
        trackedObjects->positions.push_back(position);

        geometry_msgs::TwistStamped velocity;
        velocity.twist.linear.x = velocities[0];
        velocity.twist.linear.y = velocities[1];
        velocity.twist.linear.z = velocities[2];
        velocity.header.frame_id = frame;
        velocity.header.stamp = measurementTime;
        trackedObjects->velocities.push_back(velocity);
      }

      // Step 6: Initialize filters with any remaining positions and default
      //         velocities
      for(unsigned int i = 0; i < unalignedMeasurements->points.size(); ++i){
        ROS_INFO("Initializing Kalman filter for measurement %i", i);
        boost::shared_ptr<PVFilter> filter(new PVFilter(kalmanObservationNoise, kalmanAccelerationDist));

        const PointXYZ currMeasurement = unalignedMeasurements->points[i];
        vector<double> positions(3);
        positions[0] = currMeasurement.x;
        positions[1] = currMeasurement.y;
        positions[2] = currMeasurement.z;

        vector<double> velocities(3);
        velocities[0] = initialVelocity;
        velocities[1] = initialVelocity;
        velocities[2] = initialVelocity;

        filter->init(positions, velocities, measurementTime);
        pvFilters.push_back(filter);
      }

      // Step 7: Publish the results
      if(predictedPub.getNumSubscribers() > 0 && predictedPositions->points.size() > 0){
        publishPredictedPositions(predictedPositions, measurementTime);
      }

      if(trackedObjects->positions.size() > 0){
        pub.publish(trackedObjects);

        if(markerPub.getNumSubscribers() > 0){
          publishPVArrows(trackedObjects);
        }

        if(idPub.getNumSubscribers() > 0){
          publishIDs(trackedObjects);
        }
      }
      ROS_DEBUG("Iteration end: %lu known objects", pvFilters.size());
    }

    PointCloudXYZConstPtr alignClouds(const PointCloudXYZConstPtr predictedPositions, const PointCloudXYZConstPtr measuredPositions, const PointCloudXYZPtr unalignedMeasurements) const {
  
    ROS_DEBUG("Aligning %lu points to %lu points", measuredPositions->points.size(), predictedPositions->points.size());
    
    if(predictedPositions->points.size() == 0){
      ROS_INFO("No current predicted positions");
      unalignedMeasurements->points = measuredPositions->points;
      return PointCloudXYZPtr(new PointCloudXYZ);
    }
 
    const PointXYZ nullPoint(numeric_limits<double>::infinity(), numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
  
    // If there are more measured points than predicted points, then
    // create null measurements to represent unknown objects,
    PointCloudXYZPtr measuredPositionsWithNulls(new PointCloudXYZ);
    measuredPositionsWithNulls->points = measuredPositions->points;
    for(unsigned int i = measuredPositions->points.size(); i < predictedPositions->points.size(); ++i){
      measuredPositionsWithNulls->points.push_back(nullPoint);
      ROS_DEBUG("Adding a null measured point");
    }
  
    // If the reverse is true, create null predictions.
    PointCloudXYZPtr predictedPositionsWithNulls(new PointCloudXYZ);
    predictedPositionsWithNulls->points = predictedPositions->points;
    for(unsigned int i = predictedPositions->points.size(); i < measuredPositions->points.size(); ++i){
      predictedPositionsWithNulls->points.push_back(nullPoint);
      ROS_DEBUG("Adding a null predicted point");
    }
  
    // Create a vector that will represent the ordering of the measured positions.  // Start with the lowest lexographic ordering.
    vector<unsigned int> currentOrder;
    for(unsigned int i = 0; i < measuredPositionsWithNulls->points.size(); ++i){
      currentOrder.push_back(i);
    }
  
    // Set the initial order as the best.
    vector<unsigned int> closestOrder = currentOrder;
    double lowestDistance = numeric_limits<double>::max();
    unsigned int nullsFoundForLowestDistance = 0;
  
    const double LAMBDA = 1e6;
  
    // Now search all permutations.
    do {
      // Sum the distances.
      double distanceSum = 0;
      unsigned int nullsFound = 0;
      for(unsigned int i = 0; i < currentOrder.size(); ++i){
        // Check if this is a null point and add lambda.
        if(measuredPositionsWithNulls->points[currentOrder[i]] == nullPoint || predictedPositionsWithNulls->points[i] == nullPoint){
          ROS_DEBUG("Found a null point. Adding LAMBDA.");
          distanceSum += LAMBDA;
          nullsFound++;
        }
        // Assume distances over the max correlation distance are not a match.
        else if(geometry::distance(predictedPositionsWithNulls->points[i], measuredPositionsWithNulls->points[currentOrder[i]]) > maxCorrelationDistance){
          ROS_DEBUG("Associated points were over the maximum distance %f", geometry::distance(predictedPositionsWithNulls->points[i], measuredPositionsWithNulls->points[currentOrder[i]]));
          distanceSum += LAMBDA;
          nullsFound++;
        }
        else {
          distanceSum += geometry::squaredDistance(predictedPositionsWithNulls->points[i], measuredPositionsWithNulls->points[currentOrder[i]]);
        }
      }
  
      if(distanceSum < lowestDistance){
        lowestDistance = distanceSum;
        closestOrder = currentOrder;
        nullsFoundForLowestDistance = nullsFound;
        if(distanceSum < associationEpsilon){
          ROS_DEBUG("Existing early from association search");
          break;
        }
      }
    } while(next_permutation(currentOrder.begin(), currentOrder.end()));
  
    // Remove the lambda distances from the score.
    lowestDistance -= (nullsFoundForLowestDistance * LAMBDA);
    ROS_DEBUG("Lowest total distance was %f", lowestDistance);
  
    // Excercised all perumutations.
    PointCloudXYZPtr alignedPositions(new PointCloudXYZ);
    if(lowestDistance <= associationMaxSuccessScore){
      for(unsigned int i = 0; i < measuredPositionsWithNulls->points.size(); ++i){
        const PointXYZ& currMeasurement = measuredPositionsWithNulls->points[closestOrder[i]];
        if(i >= predictedPositions->points.size()){
          ROS_DEBUG("Adding an unaligned point to the end of associated points %i %lu %lu", i, measuredPositionsWithNulls->points.size(), predictedPositions->points.size());
          unalignedMeasurements->points.push_back(currMeasurement);
        }
        else if(currMeasurement != nullPoint && geometry::distance(predictedPositionsWithNulls->points[i], currMeasurement) > maxCorrelationDistance){
          ROS_DEBUG("Points were over the maximum distance %f", geometry::distance(predictedPositionsWithNulls->points[i], currMeasurement));
          alignedPositions->points.push_back(nullPoint);
          unalignedMeasurements->points.push_back(currMeasurement);
        }
        else {
          alignedPositions->points.push_back(currMeasurement);
        }
      }
    }
    else {
      ROS_INFO("Failed to associate measurements. Score was %f.", lowestDistance);
      unalignedMeasurements->points = measuredPositions->points;
    }
    return alignedPositions;
}

   void publishPVArrows(const position_tracker::DetectedDynamicObjectsConstPtr objects) const {

     double DT_FOR_VIZ = 2;

     visualization_msgs::MarkerArrayPtr markers(new visualization_msgs::MarkerArray);
     for(unsigned int i = 0; i < objects->positions.size(); ++i){
        visualization_msgs::Marker marker;
        marker.id = i;
        marker.ns = "dod_pv_arrows";
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.header = objects->positions[i].header;
        marker.lifetime = ros::Duration(1);
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
        marker.ns = "dod_ids";
        marker.text = boost::lexical_cast<string>(i + 1);
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.header = objects->positions[i].header;
        marker.lifetime = ros::Duration(1);
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

   void publishPredictedPositions(const PointCloudXYZConstPtr predictedPositions, ros::Time stamp) const {
     visualization_msgs::MarkerArrayPtr predictedMarkers(new visualization_msgs::MarkerArray);

     for(unsigned int i = 0; i < predictedPositions->points.size(); ++i){
       visualization_msgs::Marker marker;
       marker.id = i;
       marker.ns = "dod_predicted";
       marker.action = visualization_msgs::Marker::ADD;
       marker.type = visualization_msgs::Marker::SPHERE;
       marker.header.frame_id = frame;
       marker.header.stamp = stamp;
       marker.lifetime = ros::Duration(1);
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

