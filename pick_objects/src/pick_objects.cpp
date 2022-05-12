#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Declaring variables to get
double markerPoseX = 0.0 ;
double markerPoseY = 0.0 ;

void getMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  markerPoseX = msg->pose.position.x;
  markerPoseY = msg->pose.position.y;
  ROS_INFO("check %f %f", markerPoseX, markerPoseY);
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("visualization_marker", 10, getMarkerCallback);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::spinOnce(); 
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
 
  // Define a position and orientation for the robot to reach
  // pickup zone
  goal.target_pose.pose.position.x = -3.0;
  goal.target_pose.pose.position.y = -2.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The robot reached pickup zone");
    ros::Duration(5.0).sleep();
    
    ROS_INFO("Heading towardsdropoff location");
    goal.target_pose.pose.position.x = -5.0;
    goal.target_pose.pose.position.y = -0.5;
    ac.sendGoal(goal);
    ros::Duration(5.0).sleep();
  
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
       ROS_INFO("The robot reached dropoff zone");
       ros::Duration(10.0).sleep();
    }
    else{
       ROS_INFO("Robot failed to reach droppff zone");
    }
  }
  else{
    ROS_INFO("The robot failed to reach pickup zone");
  }
  ros::Duration(5.0).sleep();

  return 0;
}
