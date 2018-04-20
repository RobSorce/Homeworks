#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "sstream"
#include "nav_msgs/Odometry.h"


void callback(const sensor_msgs::LaserScan::ConstPtr& input, ros::Publisher pub, tf::TransformListener* transform_listener){

	tf::StampedTransform latest_transform;
	ros::Time time_stamp = ros::Time::now();

	if(transform_listener->canTransform("/odom", "/base_laser_link", time_stamp, NULL)){

		transform_listener->waitForTransform("/odom", "/base_laser_link", time_stamp, ros::Duration(1.0));
		transform_listener->lookupTransform("/odom", "/base_laser_link", time_stamp, latest_transform);

		tf::Vector3 polar = latest_transform.getOrigin();
		tf::Quaternion angle = latest_transform.getRotation();

		double PosX = polar.getX();
		double PosY = polar.getY();
		double theta = angle.getAngle();
		double timestamp_Sec = latest_transform.stamp_.sec;
    double timestamp_USec = (uint64_t)latest_transform.stamp_.nsec; // - (uint64_t) latest_transform.stamp_.sec * 1e9)*1e-3;

		std_msgs::String location;

		ROS_INFO("LASER[Time: %g.%g, Position: (X: %g, Y: %g, Theta: %g)] LaserScan message received",
		timestamp_Sec, timestamp_USec, PosX, PosY, theta);

		//Ulteriori Stampe con relative info sulla posizione;
		//Disattivarle per visualizzare SOLO le stampe richieste per la soluzione.
		printf("INFO: Gathering Information.\n");
		printf("%s\n",location.data.c_str());

		pub.publish(location);

		ros::spinOnce();
	}
	else printf("WARNING: Loss of data; Information Not Available\n");

}



int main(int argc, char **argv){

	ros::init(argc, argv, "homework2");

	tf::TransformListener transform_listener;

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::String>("position", 1000);
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, boost::bind(callback, _1, pub, &transform_listener));

	ros::spin();

	return 0;
}
