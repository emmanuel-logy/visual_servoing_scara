/**
 * This module contains the class computing the Kinematics of the Scara Robot
 */

#include "ScaraKinematics.hpp"
#include <ros/param.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <map>


namespace scara
{

	Kinematics::Kinematics()	: 	m_joints_state()
	{
		// Get all link lengths
		std::map<std::string,double> link_lengths;
		ros::param::get(param_scara_links_length, link_lengths);
		m_L1 = link_lengths["L1"];
		m_L2 = link_lengths["L2"];
		m_L3 = link_lengths["L3"];
		m_L4 = link_lengths["L4"];

		// Get link width and axel_offset
		ros::param::get(param_scara_links_width, m_W);
		m_axel_offset = m_W/2;

		/*
		// Example of getting one by one
		ros::param::get("/scara/link_lengths/L1", m_L1);
		ros::param::get("/scara/link_lengths/L2", m_L2);
		ros::param::get("/scara/link_lengths/L3", m_L3);
		ros::param::get("/scara/link_width", m_W);
		m_axel_offset = m_W/2;
		*/
	}


	void Kinematics::update_joint_states(const std::vector<double>& q)
	{
		m_joints_state = q;
	}


	void Kinematics::get_joint_states(std::vector<double>& q)
	{
		q.clear();
		q = m_joints_state;
	}


	void Kinematics::fk(const std::vector<double>& q, Vector7d& final_pose)
	{
		/* For simple robots, we can compute FK */
		m_joints_state = q;

		std::vector<Eigen::Matrix4d> ht;
		link_homogeneous_matrices(ht);
		Eigen::Matrix4d T = ht[0] * ht[1] * ht[2];

		Utils::ht_to_eigenPose(T, final_pose);


		/* For complex robots, use tf2_ros package to compute FK */
		/*
		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfListener(tfBuffer);

		// Keep trying till a few frames are loaded
		while (1)
		{
			try
			{
				geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("world", "link3", ros::Time::now(), ros::Duration(3.0));
				std::cout << "*******transStamped: " << transformStamped << std::endl;
				break;
			}
			catch (tf2::TransformException &ex)
			{
				ROS_WARN("** %s",ex.what());
			}
			ros::Duration(1.0).sleep();
		}
		*/
	}


	void Kinematics::ik(const Vector7d& target_pose, std::vector<double>& q)
	{
		q.clear();

		// Goal pose of the end effector
		double xe = target_pose[0];
		double ye = target_pose[1];
		double ze = target_pose[2];

		// Making corrections with m_axel_offset information
		double L2_tmp = m_L2 - 2*m_axel_offset;

		double D = ( (xe*xe) + (ye*ye) - (L2_tmp*L2_tmp) - (m_L3*m_L3) ) / (2*L2_tmp*m_L3);
		double q2 = atan2(sqrt(1-(D*D)), D);
		static double q2_tmp = q2;

		double q1 = atan2(ye, xe) - atan2(m_L3*sin(q2), L2_tmp+(m_L3*cos(q2)));
		static double q1_tmp = q1;

		double q3 = (m_L1 + m_W + m_axel_offset) - ze;

		q.push_back(q1);
		q.push_back(q2);
		q.push_back(q3);

		/* 	The inverse kinematics mapping is typically one to many. There are usually multiple sets of
			joint variables that will yield a particular Cartesian configuration.
			Therefore, we should not verify with one particular expected numerical value.
		*/
	}


	void Kinematics::jacobian(Eigen::Matrix<double, 6, 3>& J)
	{
		std::vector<Eigen::Matrix4d> ht;
		link_homogeneous_matrices(ht);

		J = Eigen::Matrix<double, 6, 3>::Zero();
		J(0,0) = ht[0](0,2);    J(1,0) = ht[0](1,2);    J(2,0) = ht[0](2,2);
		J(0,1) = ht[1](0,2);    J(1,1) = ht[1](1,2);    J(2,1) = ht[1](2,2);
		J(0,2) = ht[2](0,2);    J(1,2) = ht[2](1,2);    J(2,2) = ht[2](2,2);
	}

	/*
	theta -> angle in radians
	d     -> distance in m
	a     -> distance in m
	alpha -> angle in radians
	*/
	Eigen::Matrix4d Kinematics::tdh(const double& theta, const double& d, const double& a, const double& alpha)
	{
		Eigen::Matrix4d dh_matrix;     // typedef Matrix<double, 4, 4> Eigen::Matrix4d;
		dh_matrix << 	cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
						sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
						0,           sin(alpha),             cos(alpha),            d,
						0,           0,                      0,                     1;
		return dh_matrix;
	}


	/* Moved out of FK function because this info is needed by jacobian function too
	 *
	 * q1  -> in rad
	 * q2  -> in rad
	 * q3  -> in m
	*/
	void Kinematics::link_homogeneous_matrices(std::vector<Eigen::Matrix4d>& ht)
	{
		double q1 = m_joints_state.at(0);
		double q2 = m_joints_state.at(1);
		double q3 = m_joints_state.at(2);

		Eigen::Matrix4d A1 = tdh(q1,                   m_L1+m_W+m_axel_offset,    	m_L2-m_axel_offset*2,   	0);
		Eigen::Matrix4d A2 = tdh(q2+(180*degToRad),    0,                  			-m_L3,                		180*degToRad);
		Eigen::Matrix4d A3 = tdh(0,                    -m_axel_offset+q3,     		0,                 			0);

		ht.clear();
		ht.push_back(A1);
		ht.push_back(A2);
		ht.push_back(A3);
	}

}    	// namespace scara
