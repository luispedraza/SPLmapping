#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#include "./SPLM/MyGLUTWindow.h"
#include "./SPLM/MapBuilder.h"

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
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
	ROS_INFO("Processing ODOMETRY...");
	float x = odom->pose.pose.position.x;
	float y = odom->pose.pose.position.y;
	double _x = odom->pose.pose.orientation.x;
	double _y = odom->pose.pose.orientation.y;
	double _z = odom->pose.pose.orientation.z;
	double _w = odom->pose.pose.orientation.w;
	float ang = atan2(2 * (_y*_x + _w*_z), _w*_w + _x*_x - _y*_y - _z*_z);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "zulumapping"); 	// Initialize ROS. Node name = zulumapping
	ros::NodeHandle n; 						// Handle to this process node
	// Subscribe to "odom" topic
	ros::Subscriber odom_sub = n.subscribe("odom", 50, odomCallback);
	//Subscribe to "base_scan" topic
	ros::Subscriber base_scan = n.subscribe("base_scan", 50, scanCallback);
	InitGLUT();
	CreateGlutWindow("My GL Window", 800, 600);

	ros::Rate rate(10); 	// 10 hz
	while (true) {
		ros::spinOnce();
		glutMainLoopEvent();
		Draw();
		rate.sleep();
	}
	return 0;
}
