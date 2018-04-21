# Homework5
Homework PCL

● Kinect point cloud filtering and visualization
Implement a ROS node that:

● Subscribes on the kinect /cloud topic

● Convert the incoming sensor_msgs/PointCloud2 messages in the pcl::PointCloud 
data structure

● Apply a voxelization in order to reduce the dimension of the cloud
 
● Filter the cloud in order to remove points outside a specified range

● Visualize the resulting point cloud using the PCL viewer
