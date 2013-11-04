#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <position_tracker/DetectedDynamicObjects.h>
#include <boost/algorithm/string/predicate.hpp>
#include <position_tracker/StartMeasurement.h>
#include <position_tracker/StopMeasurement.h>
using namespace std;

inline double square(const double a) {
    return a * a;
}

class PositionVelocityMeasurer {
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    tf::TransformListener tf;
    double totalPositionDeviation;
    double totalVelocityDeviation;

    ros::Time lastTime;
    ros::Time startTime;
    ros::Duration knownTime;
    ros::Duration unknownTime;

    auto_ptr<message_filters::Subscriber<position_tracker::DetectedDynamicObjects> > objectSub;
    message_filters::Subscriber<position_tracker::StartMeasurement> startMeasuringSub;
    message_filters::Subscriber<position_tracker::StopMeasurement> stopMeasuringSub;
    std::string modelPrefix;
    std::string groupName;

public:
    PositionVelocityMeasurer() :
        pnh("~"), totalPositionDeviation(0), totalVelocityDeviation(0),
        knownTime(0), unknownTime(0), startMeasuringSub(nh, "start_measuring", 1),
        stopMeasuringSub(nh, "stop_measuring", 1) {

        pnh.param<string>("model_prefix", modelPrefix, "ball");
        pnh.param<string>("group_name", groupName, "balls");

        // Setup the subscriber
        objectSub.reset(new message_filters::Subscriber<position_tracker::DetectedDynamicObjects>(nh,
                "object_tracks/" + groupName + "/positions_velocities", 1));
        objectSub->unsubscribe();
        objectSub->registerCallback(boost::bind(&PositionVelocityMeasurer::callback, this, _1));

        startMeasuringSub.registerCallback(boost::bind(&PositionVelocityMeasurer::startMeasuring, this, _1));
        stopMeasuringSub.registerCallback(boost::bind(&PositionVelocityMeasurer::stopMeasuring, this, _1));
    }

private:
    void startMeasuring(const position_tracker::StartMeasurementConstPtr msg){
        startTime = lastTime = msg->header.stamp;
        objectSub->subscribe();
        ROS_INFO("Measurement initiated");
    }

    void stopMeasuring(const position_tracker::StopMeasurementConstPtr msg){
        objectSub->unsubscribe();
        ROS_INFO(
                  "Measurement ended. Total Position Deviation: %f, Total Velocity Deviation: %f, Duration: %f",
                  totalPositionDeviation, totalVelocityDeviation, msg->header.stamp.toSec() - startTime.toSec());
    }

    void callback(const position_tracker::DetectedDynamicObjectsConstPtr objects) {
        ROS_DEBUG("Received a message @ %f", ros::Time::now().toSec());

        ros::Duration timePassed = objects->header.stamp - lastTime;
        lastTime = objects->header.stamp;

        // Check if there are any messages.
        if (objects->positions.size() == 0) {
            unknownTime += timePassed;
        }
        else {
            knownTime += timePassed;
        }

        double totalTime = objects->header.stamp.toSec() - startTime.toSec();
        ROS_DEBUG("Total Known Time: %f, Total Unknown Time: %f, Total Time: %f, Percent Known: %f, Percent Unknown: %f",
                  knownTime.toSec(), unknownTime.toSec(), totalTime, knownTime.toSec() / totalTime * 100,
                  unknownTime.toSec() / totalTime * 100);

        // No more work to do if object locations are unknown.
        if (objects->positions.size() == 0) {
            return;
        }

        // Fetch all the models.
        ros::service::waitForService("/gazebo/get_world_properties");
        ros::service::waitForService("/gazebo/get_model_state");

        ros::ServiceClient worldPropsServ = nh.serviceClient<gazebo_msgs::GetWorldProperties> (
                "/gazebo/get_world_properties");
        gazebo_msgs::GetWorldProperties worldProperties;
        worldPropsServ.call(worldProperties);

        vector<geometry_msgs::Point> knownPositions;
        vector<geometry_msgs::Twist> knownVelocities;

        // Iterate over all the models.
        for (unsigned int i = 0; i < worldProperties.response.model_names.size(); ++i) {
            if (!boost::starts_with(worldProperties.response.model_names[i], modelPrefix)) {
                continue;
            }

            ros::ServiceClient modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState> (
                    "/gazebo/get_model_state");
            gazebo_msgs::GetModelState modelState;
            modelState.request.model_name = worldProperties.response.model_names[i];
            modelStateServ.call(modelState);

            knownPositions.push_back(modelState.response.pose.position);
            knownVelocities.push_back(modelState.response.twist);
        }

        if (knownPositions.size() == 0) {
            ROS_INFO("No known positions");
            return;
        }

        vector<unsigned int> currentOrder;
        for (unsigned int i = 0; i < knownPositions.size(); ++i) {
            currentOrder.push_back(i);
        }

        double currPositionDeviation = numeric_limits<double>::max();
        double currVelocityDeviation = numeric_limits<double>::max();

        // Now search all permutations.
        do {
            double positionDeviation = 0;
            double velocityDeviation = 0;
            for (unsigned int i = 0; i < currentOrder.size(); ++i) {
                if (i >= objects->positions.size()) {
                    ROS_INFO("More known objects than estimated");
                    continue;
                }

                const geometry_msgs::Point& knownPosition = knownPositions[currentOrder[i]];
                const geometry_msgs::Point& estimatedPosition = objects->positions[i].point;
                positionDeviation += (square(knownPosition.x - estimatedPosition.x) + square(
                        knownPosition.y - estimatedPosition.y)) * timePassed.toSec();

                const geometry_msgs::Vector3& knownVelocity =
                        knownVelocities[currentOrder[i]].linear;
                const geometry_msgs::Vector3& estimatedVelocity =
                        objects->velocities[i].twist.linear;

                velocityDeviation += (square(knownVelocity.x - estimatedVelocity.x) + square(
                        knownVelocity.y - estimatedVelocity.y)) * timePassed.toSec();
            }

            if (positionDeviation + velocityDeviation < currPositionDeviation
                    + currVelocityDeviation) {
                currPositionDeviation = positionDeviation;
                currVelocityDeviation = velocityDeviation;
            }
        } while (next_permutation(currentOrder.begin(), currentOrder.end()));

        // Adjust deviations by number of points.
        currPositionDeviation /= knownPositions.size();
        currVelocityDeviation /= knownPositions.size();

        ROS_DEBUG("Current Position Deviation: %f, Current Velocity Deviation: %f",
                currPositionDeviation, currVelocityDeviation);

        totalPositionDeviation += currPositionDeviation;
        totalVelocityDeviation += currVelocityDeviation;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "position_velocity_measurer");
    PositionVelocityMeasurer pvm;
    ros::spin();
    return 0;
}
