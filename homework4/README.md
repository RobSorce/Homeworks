# Homework Actionlib
Homework 4

Consider the action of moving the robot forward for a certain distance with a 
desired speed.

● Define an ad­hoc ROS action:

  •Goal: desired_speed, distance (float64)
  
  •Result: odom_pose (nav_msgs/Odometry) 
  
  •Feedback: empty 
  
● Implement a SimpleActionClient that requests the execution of the action, 
  providing the desired speed and distance.
  
● Implement a SimpleActionServer that:

  •Receives the desired speed and distance values
  
  •Executes the forward motion of the robot until the requested traveled distance is reached
  
  •Returns as result the robot odometry pose at the end of the execution
