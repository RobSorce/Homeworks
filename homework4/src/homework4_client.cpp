#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <homework4/Homework4Action.h>
#include "move_base_msgs/MoveBaseActionGoal.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "homework4_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<homework4::Homework4Action> ac("homework4_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  homework4::Homework4Goal goal;
  goal.distance = 1.0; //metri
  goal.desired_speed = 0.5; //metri/sec

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout){
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    homework4::Homework4ResultConstPtr res = ac.getResult();
    nav_msgs::Odometry odom = res->odom_pose;
    ROS_INFO("Odometry pose: X: %f, Y: %f, Z: %f", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
