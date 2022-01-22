/**
 * This module contains the class managing the ROS Logistics
 */

#ifndef ROS_LOGISTICS_HPP
#define ROS_LOGISTICS_HPP

#include "ScaraConstants.hpp"
#include "ScaraKinematics.hpp"
#include "scara_gazebo/scara_ik.h"
#include "scara_gazebo/scara_fk.h"
#include "scara_gazebo/scara_vel_control.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
//#include "scara_gazebo/scara_qd_to_twist.h"
//#include "scara_gazebo/scara_twist_to_qd.h"
#include <memory>


namespace scara
{
	class ROS_Logistics
	{
	public:
		ROS_Logistics(std::shared_ptr<Kinematics>& scara_kine);

		~ROS_Logistics() = default;

		void callback_joint_states(const sensor_msgs::JointState::ConstPtr& msg);

		// Publishing this continuously to record it using rosbag and analyze end effector movements
		void pub_scara_pose(const std::vector<double>& q);

		bool srv_scara_fk(scara_gazebo::scara_fk::Request& req,
		                  scara_gazebo::scara_fk::Response& res);


		bool srv_scara_ik(scara_gazebo::scara_ik::Request& req,
		                  scara_gazebo::scara_ik::Response& res);


		bool srv_vel_control(scara_gazebo::scara_vel_control::Request& req,
							 scara_gazebo::scara_vel_control::Response& res);

	private:
        std::shared_ptr<ros::NodeHandle> m_node;	// ptr cuz we shouldn't create it before calling ros::init

        std::shared_ptr<Kinematics> m_scara_kinematics;

		ros::Subscriber m_sub_jointStates;

		ros::ServiceServer m_srv_FK;

		ros::ServiceServer m_srv_IK;

		ros::ServiceServer m_srv_VelControl;

		ros::Publisher m_pub_velCmd;

		ros::Publisher m_pub_scaraPose;

		/* Establish necessary services, clients, publisher, listener communication */
		void setUp_ROS_Communication();
	};


}    	// namespace scara




//bool srv_qd_to_twist(scara_gazebo::scara_qd_to_twist::Request  &req,
//                     scara_gazebo::scara_qd_to_twist::Response &res)
//{
//    Eigen::Matrix<float,3,1> qd = Eigen::Matrix<float,3,1>::Zero();
//    qd[0] = req.in_qd.velocity[0];
//    qd[1] = req.in_qd.velocity[1];
//    qd[2] = req.in_qd.velocity[2];
//
//    Eigen::Matrix<float, 6, 3> J;
//    jacobian(J);
//
//    Eigen::Matrix<float,6,1> twist = J * qd;
//    res.out_twist.linear.x = twist(0);
//    res.out_twist.linear.y = twist(1);
//    res.out_twist.linear.z = twist(2);
//    res.out_twist.angular.x = twist(3);
//    res.out_twist.angular.y = twist(4);
//    res.out_twist.angular.z = twist(5);
//
//    return true;
//}
//
//bool srv_twist_to_qd(scara_gazebo::scara_twist_to_qd::Request  &req,
//                     scara_gazebo::scara_twist_to_qd::Response &res)
//{
//    Eigen::Matrix<float,6,1> twist = Eigen::Matrix<float,6,1>::Zero();
//    twist[0] = req.in_twist.linear.x;
//    twist[1] = req.in_twist.linear.y;
//    twist[2] = req.in_twist.linear.z;
//    twist[3] = req.in_twist.angular.x;
//    twist[4] = req.in_twist.angular.y;
//    twist[5] = req.in_twist.angular.z;
//
//    Eigen::Matrix<float, 6, 3> J;
//    jacobian(J);
//    Eigen::Matrix<float, 3, 6> J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
//    // Eigen::Matrix<float, 3, 6>a J_pinv = J.inverse();
//
//    Eigen::Matrix<float,3,1> qd = Eigen::Matrix<float,3,1>::Zero();
//    qd = J_pinv * twist;
//    res.out_qd1 = qd(0);
//    res.out_qd2 = qd(1);
//    res.out_qd3 = qd(2);
//
//    return true;
//}

#endif	// #ifndef ROS_LOGISTICS_HPP
