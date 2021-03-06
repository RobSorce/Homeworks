#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

using namespace pcl; //mi permette di accedere a tutti i campi e strutture pcl;

//pcl::visualization::CloudViewer viewer("Cloud Viewer");


void pclCallback(const sensor_msgs::PointCloud2 pc){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pt (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelized (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(pc, *cloud_pt);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_unfiltered(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_unfiltered->setBackgroundColor (0, 0, 0);
  viewer_unfiltered->addPointCloud<pcl::PointXYZ> (cloud_pt, "Input Cloud Unfiltered" );
  viewer_unfiltered->initCameraParameters();
  viewer_unfiltered->addCoordinateSystem(1.0);
  viewer_unfiltered->spin();



  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_pt);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);


  //Voxelize the filtered cloud;
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud (cloud_filtered);
  voxel.setLeafSize (0.01, 0.01, 0.01);
  voxel.filter (*cloud_voxelized);

  //viewer.showCloud(cloud_voxelized);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_filtered (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_filtered->setBackgroundColor (0, 0, 0);
  viewer_filtered->addPointCloud<pcl::PointXYZ> (cloud_voxelized, "Filtered" );
  viewer_filtered->initCameraParameters ();
  viewer_filtered->addCoordinateSystem (1.0);
  viewer_filtered->spin();
}

int main(int argc, char **argv){

	ros::init(argc, argv, "homework5");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("camera/depth/points", 1000, pclCallback);

	ros::spin();

  return 0;
}
