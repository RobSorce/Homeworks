#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "sstream"
#include "tf/transform_listener.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

int width, height;
double scaleFactor;


void callback(const sensor_msgs::LaserScan::ConstPtr& msg){

	using namespace cv; //mi permette di accedere a tutti i campi e strutture opencv;

	int width = 800;
	int height = width;
	double scaleFactor = width/20;

	Mat img = Mat::zeros(width, height, CV_8UC3);
	img.setTo(cv::Scalar(255,255,255));

	float theta = msg->angle_min;
	float min_theta;
	float min = msg->ranges[0];
	float aux;

	Point p1 = Point((width/2) + scaleFactor*min*cos(theta), (height/2) + scaleFactor*min*sin(theta));
	Point p2;

	int dimArray = msg->ranges.size();
	for(int i = 1; i <= dimArray-1; i++){

		aux = (msg->ranges)[i];
		if (aux < min) {
			min = aux;
			min_theta = theta;
		}

	theta = theta + msg->angle_increment;

	p2 = Point((width/2) + scaleFactor*aux*cos(theta), (height/2) + scaleFactor*aux*sin(theta));

	line(img, p1, p2, Scalar(0, 0, 0), 1, LINE_8 , 0);

	p1 = p2;

	}

	circle(img, Point((width/2) + scaleFactor*min*cos(min_theta), (height/2) +
	scaleFactor*min*sin(min_theta)), 5, Scalar(0, 0, 255), 1, LINE_8, 0);

	imshow("Output window", img);

	waitKey(30);

}

int main(int argc, char **argv){

	ros::init(argc, argv, "homework3");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, callback);

	ros::spin();

  return 0;
}
