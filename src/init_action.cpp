#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

// Generated messages
#include <position_tracker/InitAction.h>

using namespace std;

/**
 * Initializes the robot for the scooping tas
 */
class InitAction {
public:
  InitAction(const string& name): as(nh, name, boost::bind(&InitAction::init, this, _1), false), actionName(name){
    
    ROS_INFO("Starting init of the init action");
    as.start();
  }
  
  /**
   * Main function to initialize the robot
   */
  void init(const position_tracker::InitGoalConstPtr& goal){
    if(!as.isActive()){
      ROS_INFO("Init action cancelled prior to start");
      return;
    }
    
    // Set the planning scene so direct arm movement can be performed
    ros::ServiceClient sceneClient = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>("/environment_server/set_planning_scene_diff");
    ROS_INFO("Waiting for planning scene service");
    sceneClient.waitForExistence();
    
    arm_navigation_msgs::SetPlanningSceneDiff::Request planningSceneReq;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planningSceneRes;

    if(!sceneClient.call(planningSceneReq, planningSceneRes)){
      ROS_WARN("Failed to set planning scene");
    }
    else {
      ROS_INFO("Planning scene set successfully");
    }
    as.setSucceeded(result);
  }
  
  protected:
    ros::NodeHandle nh;
    
    // Actionlib classes
    actionlib::SimpleActionServer<position_tracker::InitAction> as;
    string actionName;

    // create messages that are used to published feedback/result
    position_tracker::InitFeedback feedback;
    position_tracker::InitResult result;    
};

int main(int argc, char** argv){
  ros::init(argc, argv, "init_action");
  InitAction moveAction(ros::this_node::getName());
  ros::spin();
  return 0;
}
