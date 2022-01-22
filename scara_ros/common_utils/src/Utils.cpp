/**
 * This module contains the frequently used helper functions
 */

#include "Utils.hpp"

namespace scara
{

	void Utils::ht_to_rosPose(const Eigen::Matrix4d& ht, geometry_msgs::Pose& rosPose)
    {
		rosPose = {};

        Eigen::Matrix<double, 3, 3> rot_mat = ht.block<3,3>(0,0);    // Extract in_a block of matrix of size 3*3 from index [0,0] of T
        Eigen::Quaternion<double> quat(rot_mat);

        rosPose.position.x = ht(0,3);
        rosPose.position.y = ht(1,3);
        rosPose.position.z = ht(2,3);
        rosPose.orientation.x = quat.x();
        rosPose.orientation.y = quat.y();
        rosPose.orientation.z = quat.z();
        rosPose.orientation.w = quat.w();

        return;
    }

    void Utils::ht_to_eigenPose(const Eigen::Matrix4d& ht, Vector7d& eigenPose)
    {
    	eigenPose = {};

        Eigen::Matrix<double, 3, 3> rot_mat = ht.block<3,3>(0,0);    // Extract in_a block of matrix of size 3*3 from index [0,0] of T
        Eigen::Quaternion<double> quats(rot_mat);

        eigenPose << 	ht(0,3),
						ht(1,3),
						ht(2,3),
						quats.x(),
						quats.y(),
						quats.z(),
						quats.w();

        return;
    }

    void Utils::rosPose_to_eigenPose(const geometry_msgs::Pose& rosPose, Vector7d& eigenPose)
    {
    	eigenPose = Vector7d::Zero();

        eigenPose << 	rosPose.position.x,
						rosPose.position.y,
						rosPose.position.z,
						rosPose.orientation.x,
						rosPose.orientation.y,
						rosPose.orientation.z,
						rosPose.orientation.w;

        return;
    }


    void Utils::eigenPose_to_rosPose(const Vector7d& eigenPose, geometry_msgs::Pose& rosPose)
    {
        rosPose = {};

        rosPose.position.x 		=  eigenPose(0);
        rosPose.position.y 		= eigenPose(1);
        rosPose.position.z 		= eigenPose(2);
        rosPose.orientation.x 	= eigenPose(3);
        rosPose.orientation.y 	= eigenPose(4);
        rosPose.orientation.z 	= eigenPose(5);
        rosPose.orientation.w 	= eigenPose(6);

        return;
    }


}    // namespace psm_dyn
