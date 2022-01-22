/**
 * This module contains the main function
 */

#include "ROSLogistics.hpp"
#include "ScaraKinematics.hpp"
#include <memory>
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scara_gazebo", ros::init_options::AnonymousName);


	/* When the robot Kinematics is changed, just change the object name here and
	 * the rest of the code needn't be touched */
	shared_ptr<scara::Kinematics> scara_kinematics_obj = make_shared<scara::Kinematics>();


	/* This starts the ROS server to serve FK, IK, Jacobian, etc */
	scara::ROS_Logistics rosObj(scara_kinematics_obj);

    return 0;
}

