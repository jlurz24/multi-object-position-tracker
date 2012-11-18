#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

    /*
     * initialize a client of the specified type and service name.
     * @param serviceName Name of the service.
     * @return Pointer to an initialized client.
     */
    template<class T>
    static std::auto_ptr<T> initClient(const std::string& serviceName){
      ROS_INFO("Initilizing client for %s", serviceName.c_str());
      std::auto_ptr<T> client(new T(serviceName, true));

      // Wait for the action server to come up
      while(!client->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for service %s to come up", serviceName.c_str());
      }
      ROS_INFO("Service %s connected", serviceName.c_str());
      return client;
    }
  
  /*
   * Send a goal to an action client, wait for a result, and 
   * report success or failure.
   * @param client Action client to send the goal to
   * @param goal Goal to send
   * @return Whether the goal was executed successfully.
   */
  template<class T, class U>
  static bool sendGoal(const T& client, const U& goal, ros::NodeHandle& nh, double timeout = 20.0){
    bool success = false;
    if (nh.ok()){
      ROS_INFO("Sending goal");
      client->sendGoal(goal);
      if(!client->waitForResult(ros::Duration(timeout))){
        client->cancelGoal();
        ROS_INFO("Timed out achieving goal");
      }
      else {
        actionlib::SimpleClientGoalState state = client->getState();
        success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
        if(success){
          ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else {
          ROS_INFO("Action failed: %s",state.toString().c_str());
        }
    }

  }
  else {
    ROS_INFO("Nodehandle is invalid. Not sending action");
  }

  return success;
}
