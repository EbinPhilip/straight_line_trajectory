#include <line_follower.h>

int main(int argc, char **argv)
{
	// initialise ROS node
	ros::init(argc, argv, "line_follower_node");
	ros::NodeHandle n("line_follower_node");
	line_follower::line_follower lf(n);
	lf.Run();
}
