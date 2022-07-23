// ROS
#include <ros/ros.h>

// MTC massage demo implementation
#include <mr_tasks/massage_task.h>

constexpr char LOGNAME[] = "massage_demo";

int main(int argc, char** argv) {
	ros::init(argc, argv, "mr_massage_demo");
	ros::NodeHandle nh, pnh("~");
	
	// Handle Task introspection requests from RViz & feedback during execution
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	mr_tasks_demo::setupDemoScene(pnh);
	
	// Construct and run massage task
	mr_tasks_demo::MassageTask massage_task("massage_task", pnh);
	if (!massage_task.init()) {
		ROS_INFO_NAMED(LOGNAME, "Initialization failed");
		return 1;
	}
	
	if (massage_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
		if (pnh.param("execute", false)) {
			massage_task.execute();
			ROS_INFO_NAMED(LOGNAME, "Execution complete");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
	}
	
	// Keep introspection alive
	ros::waitForShutdown();
	return 0;
}
