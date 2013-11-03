#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <position_tracker/DetectedDynamicObjects.h>
#include <boost/algorithm/string/predicate.hpp>

using namespace std;

inline double square(const double a) {
    return a * a;
}

class PositionVelocityMeasurer {
private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    tf::TransformListener tf;
    double averagePositionDeviation;
    double averageVelocityDeviation;
    unsigned int iterations;
    ros::Time startTime;
    message_filters::Subscriber<position_tracker::DetectedDynamicObjects> objectSub;

public:
    PositionVelocityMeasurer() :
        privateHandle("~"), averagePositionDeviation(0), averageVelocityDeviation(0),
                iterations(0), startTime(ros::Time::now()),
                objectSub(nh, "object_tracks/balls/positions_velocities", 1) {
        objectSub.registerCallback(boost::bind(&PositionVelocityMeasurer::callback, this, _1));
        ROS_INFO("Measurement initiated");
    }

    ~PositionVelocityMeasurer() {
        ROS_INFO("Measurement ended");
    }

private:
    void callback(const position_tracker::DetectedDynamicObjectsConstPtr objects) {
        ROS_DEBUG("Received a message @ %f", ros::Time::now().toSec());

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
            if (!boost::starts_with(worldProperties.response.model_names[i], "ball")) {
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
                positionDeviation += sqrt(
                        square(knownPosition.x - estimatedPosition.x) + square(
                                knownPosition.y - estimatedPosition.y));

                const geometry_msgs::Vector3& knownVelocity =
                        knownVelocities[currentOrder[i]].linear;
                const geometry_msgs::Vector3& estimatedVelocity =
                        objects->velocities[i].twist.linear;

                velocityDeviation += sqrt(
                        square(knownVelocity.x - estimatedVelocity.x) + square(
                                knownVelocity.y - estimatedVelocity.y));
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

        // Probably should divide by the number of points.
        ROS_INFO("Current Position Deviation: %f, Current Velocity Deviation: %f",
                currPositionDeviation, currVelocityDeviation);

        // Update the moving averages.
        averagePositionDeviation = (currPositionDeviation + iterations * averagePositionDeviation)
                / (iterations + 1);
        averageVelocityDeviation = (currVelocityDeviation + iterations * averageVelocityDeviation)
                / (iterations + 1);
        iterations++;

        double duration = std::max(ros::Time::now().toSec() - startTime.toSec(), 0.1);
        ROS_INFO(
                "Average Position Deviation: %f, Average Velocity Deviation: %f, iterations %u, Duration: %f, F/s: %f",
                averagePositionDeviation, averageVelocityDeviation, iterations, duration,
                iterations / duration);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "position_velocity_measurer");
    PositionVelocityMeasurer pvm;
    ros::spin();
    return 0;
}
