#include <ros/ros.h>  			// Includes headers neccessary to use the most common pieces of ROS
#include <nav_msgs/Odometry.h>  // We include the nav_msgs/Odometry message that we want to send over the wire
#include <tf/transform_broadcaster.h> 
#include <tf/transform_listener.h> 
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/LaserScan.h> 
#include <math.h>

#include <newmat/newmat.h>
using namespace NEWMAT;

#include "./SPLM/MyGLUTWindow.h"
#include "./SPLM/MapBuilder.h"

// The Map Builder
CMapBuilder builder;


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& base_scan) {
	ROS_INFO("Processing LASER...");
	std::vector<float> scandata(base_scan->ranges.size());
	for (unsigned int k = 0; k < base_scan->ranges.size(); k++) {
		scandata[k] = base_scan->ranges[k];
	}
	float rmin = base_scan->range_min;
	float rmax = base_scan->range_max;
	float amin = base_scan->angle_min;
	float amax = base_scan->angle_max;
	float step = base_scan->angle_increment;
	builder.ProcessLaser(scandata, rmin, rmax, amin, amax, step);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
	ROS_INFO("Processing ODOMETRY...");
	float x = odom->pose.pose.position.x;
	float y = odom->pose.pose.position.y;
	double _x = odom->pose.pose.orientation.x;
	double _y = odom->pose.pose.orientation.y;
	double _z = odom->pose.pose.orientation.z;
	double _w = odom->pose.pose.orientation.w;
	float ang = atan2(2 * (_y * _x + _w * _z), _w * _w + _x * _x - _y * _y - _z
			* _z);
	builder.ProcessOdometry(x, y, ang);
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "SPLmapping"); // Initialize ROS. Node name = SPLmapping
	ros::NodeHandle n; // Handle to this process node
	// Subscribe to "odom" topic
	ros::Subscriber odom_sub = n.subscribe("odom", 50, odomCallback);
	//Subscribe to "base_scan" topic
	ros::Subscriber base_scan = n.subscribe("base_scan", 50, scanCallback);

	InitGLUT();
	CreateGlutWindow("My GL Window", 800, 600);
	glSetMapBuilder(&builder);

	//glutMainLoop();
	//ros::spin();
	ros::Rate rate(10); // 10 hz
	while (true) {
		ros::spinOnce();
		glutMainLoopEvent();
		Draw();
		rate.sleep();
	}
	return 0;
}
