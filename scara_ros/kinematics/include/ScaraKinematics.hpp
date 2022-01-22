/**
 * This module contains the class computing the Kinematics of the Scara Robot
 */

#ifndef SCARA_KINEMATICS_HPP
#define SCARA_KINEMATICS_HPP

#include "ScaraConstants.hpp"
#include "Utils.hpp"
#include <vector>


namespace scara
{
	class Kinematics
	{
	public:
		Kinematics();

		~Kinematics() = default;

		void update_joint_states(const std::vector<double>& q);

		void get_joint_states(std::vector<double>& q);

		void fk(const std::vector<double>& q, Vector7d& final_pose);

		void ik(const Vector7d& target_pose, std::vector<double>& q);

		void jacobian(Eigen::Matrix<double, 6, 3>& J);

	private:
		// Robot diemensions
		double m_L1;
		double m_L2;
		double m_L3;
		double m_L4;
		double m_W;
		double m_axel_offset;

		std::vector<double> m_joints_state;


		/*
		theta -> angle in radians
		d     -> distance in m
		a     -> distance in m
		alpha -> angle in radians
		*/
		Eigen::Matrix4d tdh(const double& theta, const double& d, const double& a, const double& alpha);

		/* Moved out of FK function because this info is needed by jacobian function too
		 *
		 * q1  -> in rad
		 * q2  -> in rad
		 * q3  -> in m
		*/
		void link_homogeneous_matrices(std::vector<Eigen::Matrix4d>& ht);
	};

}    	// namespace scara

#endif	// #ifndef SCARA_KINEMATICS_HPP
