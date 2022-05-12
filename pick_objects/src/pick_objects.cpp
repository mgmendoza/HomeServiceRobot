#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void getMarkerCallback(const std_msgs::String::ConstPtr& msg)
{
  // The pose is within the _msg object
  ROS_INFO_STREAM("pose: " << _msg->pose);
  ROS_INFO_STREAM("marker.position.x: " << _msg->marker.position.x);
  ROS_INFO_STREAM("marker.position.y: " << _msg->marker.position.y);
  ROS_INFO_STREAM("marker.position.z: " << _msg->marker.position.z);

}
int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("visualization_marker", 1000, getMarkerCallback);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -6.0;
  goal.target_pose.pose.orientation.w = -2.5;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot reached pickup zone");
  else
    ROS_INFO("The robot failed to reach pickup zone");

  return 0;
}
