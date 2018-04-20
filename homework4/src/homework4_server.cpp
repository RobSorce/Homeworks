#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <homework4/Homework4Action.h>
#include <math.h>

class Homework4Action
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<homework4::Homework4Action> as_;
  //NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  homework4::Homework4Feedback feedback_;
  homework4::Homework4Result result_;

  ros::Subscriber odom_subscriber;
  bool success;
  bool odom_set;
  bool preempted;
  float desired_speed;
  float distance_moved;
  float x;
  float y;
  float z;

public:

  Homework4Action(std::string name) :
    as_(nh_, name, boost::bind(&Homework4Action::executeCB, this, _1), false),
    action_name_(name)
  {
    success = false;
		odom_set = false;
		distance_moved = -1.0;
    odom_subscriber = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &Homework4Action::Callback, this);
    as_.start();
  }

  ~Homework4Action(void)
  {
  }

  void Callback(const nav_msgs::Odometry::ConstPtr& msg){
    float actual_x = msg->pose.pose.position.x;
    float actual_y = msg->pose.pose.position.y;
    float actual_z = msg->pose.pose.position.z;
    if(!odom_set) {
      x = actual_x;
      y = actual_y;
      z = actual_z;
      odom_set = true;
    }
    else if(distance_moved >= 0){
      if(sqrt(pow(actual_x-x,2) + pow(actual_y-y,2) + pow(actual_z-z,2)) >= distance_moved) {
        success = true;
        result_.odom_pose = *msg;
      }
    }
  }

  void executeCB(const homework4::Homework4GoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(30);

    distance_moved = goal->distance;
		desired_speed = goal->desired_speed;

    geometry_msgs::Twist new_msg;
		ros::Publisher pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, desired_speed: %f, distance: %f", action_name_.c_str(), desired_speed, distance_moved);

    // start executing the action
    while(!success){

      ROS_INFO("Distance moved: %f", distance_moved);

      new_msg.linear.x = desired_speed;
      pub.publish(new_msg);

      ROS_INFO("desired_speed %f", desired_speed);
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      // r.sleep();
    }

    if(success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    success = false;
		odom_set = false;
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "homework4_server");

  Homework4Action homework4("homework4_server");
  ros::spin();

  return 0;
}
