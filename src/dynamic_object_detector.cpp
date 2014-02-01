#include <ros/ros.h>
#include "pv_filter.h"
#include <message_filters/subscriber.h>
#include <position_tracker/DetectedObjects.h>
#include <position_tracker/DetectedDynamicObjects.h>
#include <position_tracker/DetectedDynamicObject.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <pcl/common/geometry.h>
#include <google/profiler.h>

#define ENABLE_PROFILING 0

namespace {
using namespace std;
using namespace pcl;

typedef PointCloud<PointXYZ> PointCloudXYZ;
typedef PointCloudXYZ::ConstPtr PointCloudXYZConstPtr;
typedef PointCloudXYZ::Ptr PointCloudXYZPtr;
typedef vector<boost::shared_ptr<PVFilter> > PVFilterVector;

inline Eigen::Vector3f operator-(const PointXYZ& p1, const PointXYZ& p2) {
    return Eigen::Vector3f(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

inline bool operator==(const PointXYZ& p1, const PointXYZ& p2) {
    return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
}

inline bool operator!=(const PointXYZ& p1, const PointXYZ&p2) {
    return !(p1 == p2);
}

struct EraseStale {

    ros::Duration filterStaleThreshold;
    ros::Time measurementTime;

    EraseStale(const ros::Duration& aFilterStaleThreshold, const ros::Time& aMeasurementTime) :
            filterStaleThreshold(aFilterStaleThreshold), measurementTime(aMeasurementTime) {
    }

    bool operator()(const boost::shared_ptr<PVFilter>& filter) {
        if (measurementTime - filter->getLastUpdate() > filterStaleThreshold) {
            ROS_DEBUG(
                    "Pruning a filter that has not been updated since %f", filter->getLastUpdate().toSec());
            return true;
        }
        return false;
    }
};

struct LastUpdateSorter {
    bool operator()(const boost::shared_ptr<PVFilter>& a, const boost::shared_ptr<PVFilter>& b) const {
        return a->getLastUpdate() > b->getLastUpdate();
    }
};

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
    ros::Duration filterStaleThreshold;
    double maxCorrelationDistance;
    int maxFilters;
    ros::Publisher pub;
    ros::Publisher markerPub;
    ros::Publisher predictedPub;
    ros::Publisher idPub;
    bool subscribed;

public:
    DynamicObjectDetector() :
            privateHandle("~"), kalmanObservationNoise(0.0), kalmanAccelerationDist(0.0), subscribed(false) {
        privateHandle.param<string>("object_name", objectName, "dog");
        // TODO: Consider switching to /map
        privateHandle.param<string>("frame", frame, "/base_footprint");
        privateHandle.param<double>("initial_velocity", initialVelocity, 0.0);

        // Kalman filter parameters
        privateHandle.param<double>("kalman_observation_noise", kalmanObservationNoise, 0.05);
        privateHandle.param<double>("kalman_acceleration_dist", kalmanAccelerationDist, 0.1);
        privateHandle.param<double>("association_epsilon", associationEpsilon, 1e-6);
        privateHandle.param<double>("association_max_success_score", associationMaxSuccessScore,
                2.0);
        double filterStaleThresholdD;
        privateHandle.param<double>("filter_stale_threshold", filterStaleThresholdD, 1.0);
        filterStaleThreshold = ros::Duration(filterStaleThresholdD);
        privateHandle.param<double>("max_correlation_distance", maxCorrelationDistance, 5.0);
        privateHandle.param<int>("max_filters", maxFilters, 3);

        ROS_DEBUG("Tracking objects with object name %s", objectName.c_str());

        ros::SubscriberStatusCallback connectCB = boost::bind(
                &DynamicObjectDetector::startListening, this);
        ros::SubscriberStatusCallback disconnectCB = boost::bind(
                &DynamicObjectDetector::stopListening, this);

        pub = nh.advertise<position_tracker::DetectedDynamicObjects>(
                "object_tracks/" + objectName + "/positions_velocities", 1, connectCB,
                disconnectCB);

        markerPub = nh.advertise<visualization_msgs::MarkerArray>(
                "object_tracks/" + objectName + "/markers", 1, connectCB, disconnectCB);

        predictedPub = nh.advertise<visualization_msgs::MarkerArray>(
                "object_tracks/" + objectName + "/predicted_markers", 1, connectCB, disconnectCB);

        idPub = nh.advertise<visualization_msgs::MarkerArray>(
                "object_tracks/" + objectName + "/identities", 1, connectCB, disconnectCB);

        ROS_DEBUG("Initialization of the dynamic object detector complete");
    }

private:
    void stopListening() {
        if (pub.getNumSubscribers() == 0 && markerPub.getNumSubscribers() == 0
                && predictedPub.getNumSubscribers() == 0 && idPub.getNumSubscribers() == 0) {
            ROS_DEBUG("Stopping listeners for multi object detector");
            subscribed = false;
            objectsSub->unsubscribe();
        }
    }

    void startListening() {
        if (subscribed) {
            return;
        }

        ROS_DEBUG("Starting to listen for detected object messages");
        subscribed = true;
        if (objectsSub.get() == NULL) {
            objectsSub.reset(
                    new message_filters::Subscriber<position_tracker::DetectedObjects>(nh,
                            "/object_locations/" + objectName, 1));
            objectsSub->registerCallback(
                    boost::bind(&DynamicObjectDetector::detectedObjectsCallback, this, _1));
        }
        else {
            objectsSub->subscribe();
        }
        ROS_DEBUG("Registration for detected objects event complete.");
    }

    void detectedObjectsCallback(const position_tracker::DetectedObjectsConstPtr objects) {

        ROS_DEBUG("Received detected objects message with %lu objects. Currently %lu known objects", objects->positions.size(), pvFilters.size());

        if (objects->positions.size() != pvFilters.size()) {
            ROS_DEBUG(
                    "Received %lu measurements in frame %s (dynamic object detector frame is: %s) and %lu have filters", objects->positions.size(), objects->header.frame_id.c_str(), frame.c_str(), pvFilters.size());
        }

        // Confirm frames are correct.
        for (unsigned int i = 0; i < objects->positions.size(); ++i) {
            ROS_DEBUG("Message %u is in frame %s (dynamic object detector frame is: %s)", i, objects->positions[i].header.frame_id.c_str(), frame.c_str());
            if (objects->positions[i].header.frame_id != frame) {
                ROS_ERROR(
                        "Frame does not match: %s", objects->positions[i].header.frame_id.c_str());
                return;
            }
        }

        // Step 1: Predict the new positions
        ros::Time measurementTime = objects->header.stamp;
        PointCloudXYZPtr predictedPositions(new PointCloudXYZ);
        PointCloudXYZPtr predictedVelocities(new PointCloudXYZ);

        for (unsigned int i = 0; i < pvFilters.size(); ++i) {
            vector<double> positions;
            vector<double> velocities;

            pvFilters.at(i)->predict(positions, velocities, measurementTime);
            ROS_DEBUG("Position %u predicted @ %f %f %f with velocities %f %f %f", i, positions[0], positions[1], positions[2], velocities[0], velocities[1], velocities[2]);
            predictedPositions->points.push_back(
                    PointXYZ(positions[0], positions[1], positions[2]));
            predictedVelocities->points.push_back(
                    PointXYZ(velocities[0], velocities[1], velocities[2]));

        }

        // Step 2: Create a point cloud from the detected object centers.
        PointCloudXYZPtr measuredPositions(new PointCloudXYZ);
        for (unsigned int i = 0; i < objects->positions.size(); ++i) {
            PointXYZ point(objects->positions[i].point.x, objects->positions[i].point.y,
                    objects->positions[i].point.z);
            measuredPositions->points.push_back(point);
        }

        // Step 3: Associate the predicted points with the measurements.
        const PointCloudXYZPtr unalignedMeasurements(new PointCloudXYZ);
        const PointCloudXYZConstPtr final = alignClouds(predictedPositions, measuredPositions,
                unalignedMeasurements);

        // Step 4: Update the associated filters with the new measurements.
        assert(final->points.size() == pvFilters.size());
        assert(final->points.size() == predictedPositions->size());

        for (unsigned int i = 0; i < final->points.size(); ++i) {
            // Ignore null measurements.
            if (final->points[i].x != numeric_limits<double>::infinity()) {
                vector<double> point(3);
                point[0] = final->points[i].x;
                point[1] = final->points[i].y;
                point[2] = final->points[i].z;
                ROS_DEBUG("Applying measurement to point %u with values %f %f %f", i, point[0], point[1], point[2]);
                pvFilters.at(i)->measure(point, measurementTime);
            }
        }

        // Step 4a: Prune any filters that have not been updated
        //          recently.
        pvFilters.erase(
                std::remove_if(pvFilters.begin(), pvFilters.end(),
                        EraseStale(filterStaleThreshold, measurementTime)), pvFilters.end());

        // Step 5: Get the current estimates for the state variables
        position_tracker::DetectedDynamicObjectsPtr trackedObjects(
                new position_tracker::DetectedDynamicObjects);
        trackedObjects->header.frame_id = frame;
        trackedObjects->header.stamp = measurementTime;

        for (unsigned int i = 0; i < pvFilters.size(); ++i) {
            geometry_msgs::PointStamped position;
            position.point.x = predictedPositions->points[i].x;
            position.point.y = predictedPositions->points[i].y;
            position.point.z = predictedPositions->points[i].z;
            position.header.frame_id = frame;
            position.header.stamp = measurementTime;

            geometry_msgs::TwistStamped velocity;
            velocity.twist.linear.x = predictedVelocities->points[i].x;
            velocity.twist.linear.y = predictedVelocities->points[i].y;
            velocity.twist.linear.z = predictedVelocities->points[i].z;
            velocity.header.frame_id = frame;
            velocity.header.stamp = measurementTime;

            position_tracker::DetectedDynamicObject obj;
            obj.position = position;
            obj.velocity = velocity;
            obj.measuredTime = pvFilters.at(i)->getLastUpdate();
            obj.id = pvFilters.at(i)->getId();
            trackedObjects->objects.push_back(obj);
        }

        // Step 6: Prune filters to reduce the number of filters to at most
        // max - new filters.
        int filtersToKeep = std::max(maxFilters - static_cast<int>(unalignedMeasurements->points.size()), 0);
        if(filtersToKeep < static_cast<int>(pvFilters.size())){
            // Sort pvFilters by last updated.
            ROS_INFO("Currently %lu filters. Keeping %u filters for %lu new measurements", pvFilters.size(), filtersToKeep, unalignedMeasurements->points.size());
            sort(pvFilters.begin(), pvFilters.end(), LastUpdateSorter());
            pvFilters.erase(pvFilters.begin() + filtersToKeep, pvFilters.end());
            ROS_INFO("After removal %lu filters", pvFilters.size());
        }

        // Step 7: Initialize filters with any remaining positions and default
        //         velocities
        for (unsigned int i = 0; i < unalignedMeasurements->points.size(); ++i) {
            boost::shared_ptr<PVFilter> filter(
                    new PVFilter(kalmanObservationNoise, kalmanAccelerationDist));

            const PointXYZ currMeasurement = unalignedMeasurements->points[i];
            vector<double> positions(3);
            positions[0] = currMeasurement.x;
            positions[1] = currMeasurement.y;
            positions[2] = currMeasurement.z;

            vector<double> velocities(3);
            velocities[0] = initialVelocity;
            velocities[1] = initialVelocity;
            velocities[2] = initialVelocity;

            ROS_INFO("Initializing Kalman filter for measurement %i with position %f %f %f", i, positions[0], positions[1], positions[2]);

            filter->init(positions, velocities, measurementTime);
            pvFilters.push_back(filter);
        }

        // Step 7: Publish the results
        if (predictedPub.getNumSubscribers() > 0 && predictedPositions->points.size() > 0) {
            publishPredictedPositions(predictedPositions, measurementTime);
        }

        pub.publish(trackedObjects);

        if (markerPub.getNumSubscribers() > 0) {
            publishPVArrows(trackedObjects);
        }

        if (idPub.getNumSubscribers() > 0) {
            publishIDs(trackedObjects);
        }
        ROS_DEBUG("Iteration end: %lu known objects", pvFilters.size());
    }

    PointCloudXYZConstPtr alignClouds(const PointCloudXYZConstPtr predictedPositions,
            const PointCloudXYZConstPtr measuredPositions,
            const PointCloudXYZPtr unalignedMeasurements) const {

        ROS_INFO(
                "Aligning %lu points to %lu points", measuredPositions->points.size(), predictedPositions->points.size());

        if (predictedPositions->points.size() == 0) {
            ROS_DEBUG("No current predicted positions");
            unalignedMeasurements->points = measuredPositions->points;
            return PointCloudXYZPtr(new PointCloudXYZ);
        }

        const PointXYZ nullPoint(numeric_limits<double>::infinity(),
                numeric_limits<double>::infinity(), numeric_limits<double>::infinity());

        // If there are more measured points than predicted points, then
        // create null measurements to represent unknown objects,
        PointCloudXYZPtr measuredPositionsWithNulls(new PointCloudXYZ);
        measuredPositionsWithNulls->points = measuredPositions->points;
        for (unsigned int i = measuredPositions->points.size();
                i < predictedPositions->points.size(); ++i) {
            measuredPositionsWithNulls->points.push_back(nullPoint);
        }

        // If the reverse is true, create null predictions.
        PointCloudXYZPtr predictedPositionsWithNulls(new PointCloudXYZ);
        predictedPositionsWithNulls->points = predictedPositions->points;
        for (unsigned int i = predictedPositions->points.size();
                i < measuredPositions->points.size(); ++i) {
            predictedPositionsWithNulls->points.push_back(nullPoint);
        }

        if(measuredPositions->points.size() == 0){
            ROS_DEBUG("No measured positions. Returning unaligned cloud.");
            return measuredPositionsWithNulls;
        }

        /*
         * Create a vector that will represent the ordering of the measured positions.
         * Start with the lowest lexographic ordering.
         */
        vector<unsigned int> currentOrder;
        for (unsigned int i = 0; i < measuredPositionsWithNulls->points.size(); ++i) {
            currentOrder.push_back(i);
        }

        ROS_INFO("Initializing correlation. Size of current order vector is: %lu", currentOrder.size());

        // Set the initial order as the best.
        vector<unsigned int> closestOrder = currentOrder;
        double lowestDistance = numeric_limits<double>::max();
        unsigned int nullsFoundForLowestDistance = 0;

        const double LAMBDA = 1e6;
        unsigned int iterations = 0;
        // Now search all permutations.
        do {
            ROS_DEBUG("Checking next iteration number: %u", iterations);
            // Sum the distances.
            double distanceSum = 0;
            unsigned int nullsFound = 0;
            for (unsigned int i = 0; i < currentOrder.size(); ++i) {
                // Check if this is a null point and add lambda.
                if (measuredPositionsWithNulls->points[currentOrder[i]] == nullPoint
                        || predictedPositionsWithNulls->points[i] == nullPoint) {
                    distanceSum += LAMBDA;
                    nullsFound++;
                }
                // Assume distances over the max correlation distance are not a match.
                else if (geometry::distance(predictedPositionsWithNulls->points[i],
                        measuredPositionsWithNulls->points[currentOrder[i]])
                        > maxCorrelationDistance) {
                    ROS_DEBUG(
                            "Associated points were over the maximum distance %f", geometry::distance(predictedPositionsWithNulls->points[i], measuredPositionsWithNulls->points[currentOrder[i]]));
                    distanceSum += LAMBDA;
                    nullsFound++;
                }
                else {
                    distanceSum += geometry::squaredDistance(predictedPositionsWithNulls->points[i],
                            measuredPositionsWithNulls->points[currentOrder[i]]);
                }
            }

            if (distanceSum < lowestDistance) {
                lowestDistance = distanceSum;
                closestOrder = currentOrder;
                nullsFoundForLowestDistance = nullsFound;
                if (distanceSum < associationEpsilon) {
                    ROS_DEBUG("Existing early from association search");
                    break;
                }
            }
            ++iterations;
        } while (next_permutation(currentOrder.begin(), currentOrder.end()));

        // Remove the lambda distances from the score.
        lowestDistance -= (nullsFoundForLowestDistance * LAMBDA);

        // Allow the lowest distance to be relative to the number of tracked objects.
        lowestDistance /= static_cast<double>(measuredPositionsWithNulls->points.size());

        ROS_DEBUG("Lowest total distance was %f", lowestDistance);

        // Exercised all permutations.
        PointCloudXYZPtr alignedPositions(new PointCloudXYZ);
        if (lowestDistance <= associationMaxSuccessScore) {
            for (unsigned int i = 0; i < measuredPositionsWithNulls->points.size(); ++i) {
                const PointXYZ& currMeasurement =
                        measuredPositionsWithNulls->points[closestOrder[i]];
                if (i >= predictedPositions->points.size()) {
                    ROS_DEBUG(
                            "Adding an unaligned point to the end of associated points %i %lu %lu", i, measuredPositionsWithNulls->points.size(), predictedPositions->points.size());
                    unalignedMeasurements->points.push_back(currMeasurement);
                }
                else if (currMeasurement != nullPoint
                        && geometry::distance(predictedPositionsWithNulls->points[i],
                                currMeasurement) > maxCorrelationDistance) {
                    ROS_DEBUG(
                            "Points were over the maximum distance %f", geometry::distance(predictedPositionsWithNulls->points[i], currMeasurement));
                    alignedPositions->points.push_back(nullPoint);
                    unalignedMeasurements->points.push_back(currMeasurement);
                }
                else {
                    ROS_DEBUG("Adding aligned measurement %u", i);
                    alignedPositions->points.push_back(currMeasurement);
                }
            }
        }
        else {
            ROS_INFO( "Failed to associate measurements. Score was %f.", lowestDistance);
            unalignedMeasurements->points = measuredPositions->points;
            // Set null measurements for all the filters.
            alignedPositions->points.resize(predictedPositions->points.size());
            std::fill(alignedPositions->points.begin(), alignedPositions->points.end(), nullPoint);
        }
        return alignedPositions;
    }

    void publishPVArrows(const position_tracker::DetectedDynamicObjectsConstPtr objects) const {

        double DT_FOR_VIZ = 2;

        visualization_msgs::MarkerArrayPtr markers(new visualization_msgs::MarkerArray);
        for (unsigned int i = 0; i < objects->objects.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.id = objects->objects[i].id;
            marker.ns = "dod_pv_arrows";
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.header = objects->objects[i].position.header;
            marker.lifetime = ros::Duration(1);
            marker.points.resize(2);
            marker.points[0] = objects->objects[i].position.point;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.04;
            marker.color.a = 1;
            marker.color.r = 1;
            marker.color.g = 1;
            marker.color.b = 0;

            // Calculate the next point
            marker.points[1] = objects->objects[i].position.point;
            marker.points[1].x += objects->objects[i].velocity.twist.linear.x * DT_FOR_VIZ;
            marker.points[1].y += objects->objects[i].velocity.twist.linear.y * DT_FOR_VIZ;
            marker.points[1].z += objects->objects[i].velocity.twist.linear.z * DT_FOR_VIZ;
            markers->markers.push_back(marker);
        }
        markerPub.publish(markers);
    }

    void publishIDs(const position_tracker::DetectedDynamicObjectsConstPtr objects) const {
        visualization_msgs::MarkerArrayPtr markers(new visualization_msgs::MarkerArray);

        for (unsigned int i = 0; i < objects->objects.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.id = objects->objects[i].id;
            marker.ns = "dod_ids";
            marker.text = boost::lexical_cast<string>(i + 1);
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.header = objects->objects[i].position.header;
            marker.lifetime = ros::Duration(1);
            marker.color.a = 1;
            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 1;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
            marker.pose.position = objects->objects[i].position.point;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1;
            markers->markers.push_back(marker);
        }
        idPub.publish(markers);
    }

    void publishPredictedPositions(const PointCloudXYZConstPtr predictedPositions,
            ros::Time stamp) const {
        visualization_msgs::MarkerArrayPtr predictedMarkers(new visualization_msgs::MarkerArray);

        for (unsigned int i = 0; i < predictedPositions->points.size(); ++i) {
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
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_object_detector");
#if ENABLE_PROFILING == 1
    ProfilerStart(("/tmp/" + ros::this_node::getName() +".prof").c_str());
#endif
    DynamicObjectDetector dobd;
    ros::spin();
#if ENABLE_PROFILING == 1
    ProfilerStop();
    ProfilerFlush();
#endif
    return 0;
}

