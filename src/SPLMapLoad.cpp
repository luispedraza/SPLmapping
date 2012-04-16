#include <ros/ros.h>  			// Includes headers necessary to use the most common pieces of ROS
#include <ros/package.h>
#include "./SPLM/MyGLUTWindow.h"
#include "./SPLM/MapBuilder.h"

// The Map Builder
CMapBuilder builder;

int main(int argc, char **argv) {

	ros::init(argc, argv, "SPLMapLoad"); // Initialize ROS. Node name = SPLmapping
	ros::NodeHandle n; // Handle to this process node
	if (argc != 2) {
		cout << "No map provided" << endl;
		return 0;
	}

	std::string path = ros::package::getPath("SPLmapping");
	path += "/maps/";
	path += argv[1];
	cout << "Loading map: " << path << endl;
	if (builder.m_Map != NULL)
		delete builder.m_Map;
	builder.m_Map = new CMapSP;
	std::ifstream file(path.data());
	builder.m_Map->Load(file);
	builder.m_Mlist.clear();
	builder.m_Mlist.push_back(builder.m_Map);
	builder.m_MatchMap = builder.m_Map;

	InitGLUT();
	CreateGlutWindow("My GL Window", 800, 600);
	glSetMapBuilder(&builder);

	//glutMainLoop();
	//ros::spin();
	ros::Rate rate(10); // 10 hz
	while (true) {
		//ros::spinOnce();
		glutMainLoopEvent();
		Draw();
		rate.sleep();
	}
	return 0;
}
