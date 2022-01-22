/**
 * This module contains the frequently used helper functions
 */

#ifndef UTILS_HPP
#define UTILS_HPP

#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>

namespace scara
{

	// Some helpful Eigen typedefs
	using Vector7d = Eigen::Matrix<double, 7, 1>;


    class Utils
    {
    public:
        static void ht_to_rosPose(const Eigen::Matrix4d& ht, geometry_msgs::Pose& pose);

        static void ht_to_eigenPose(const Eigen::Matrix4d& ht, Vector7d& pose);

        static void rosPose_to_eigenPose(const geometry_msgs::Pose& rosPose, Vector7d& eigenPose);

        static void eigenPose_to_rosPose(const Vector7d& eigenPose, geometry_msgs::Pose& rosPose);

    private:

    };

}    	// namespace scara

#endif	// #ifndef UTILS_HPP
