#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <position_tracker/utils.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  std::auto_ptr<TrajClient> traj_client_;
  ros::NodeHandle nh;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() {
    traj_client_ = initClient<TrajClient>("l_arm_controller/joint_trajectory_action");
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
  {
    
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    sendGoal(traj_client_, goal, nh);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("l_wrist_roll_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // Positions
    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].positions[0] = 2.0;
    goal.trajectory.points[0].positions[3] = -2.05;
    goal.trajectory.points[0].positions[5] = -0.1;
   
    // Velocities
    goal.trajectory.points[0].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[0].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
    return goal;
  }

};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  RobotArm arm;

  // Start the trajectory. This will not return until completion.
  arm.startTrajectory(arm.armExtensionTrajectory());
}

