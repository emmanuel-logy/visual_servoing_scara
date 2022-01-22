/**
 * This module contains the main function
 */

#include "ROSLogistics_VisualServoing.hpp"
#include <memory>
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scara_visual", ros::init_options::AnonymousName);

	cout << "start" << endl;
	/* This starts the ROS server to perform Visual Servoing */
	scara::ROSLogistics_VisualServoing rosObj;
	cout << "end" << endl;

    return 0;
}

