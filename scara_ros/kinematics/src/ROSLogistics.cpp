/**
 * This module contains the class managing the ROS Logistics
 */

#include "ROSLogistics.hpp"
#include "Utils.hpp"
#include "ScaraConstants.hpp"
#include "ScaraKinematics.hpp"
#include "scara_gazebo/Float64Array.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
//#include "scara_gazebo/scara_qd_to_twist.h"
//#include "scara_gazebo/scara_twist_to_qd.h"


namespace scara
{

	ROS_Logistics::ROS_Logistics(std::shared_ptr<Kinematics>& scara_kine)	:	m_node(nullptr),
																				m_scara_kinematics(nullptr)
	{
		// Register the node with ROS Master
		m_node = std::make_shared<ros::NodeHandle>();


		// Establish all necessary connections
		setUp_ROS_Communication();
		ROS_INFO("scara_gazebo all services and publishers up and running . . .");


		// Create Kinematics object after brining up ros parameter server
		m_scara_kinematics = scara_kine;

		// Keep looking into service, subscriber queues once in 100ms
		ros::Rate rate(spin_rate);	// 10 Hz = 1/10 sec = 100ms
		while (m_node->ok())
		{
			ros::spinOnce();
			rate.sleep();
		}
	}


	void ROS_Logistics::callback_joint_states(const sensor_msgs::JointState::ConstPtr& msg)
	{
		m_scara_kinematics->update_joint_states(msg->position);
		pub_scara_pose(msg->position);
	}


	void ROS_Logistics::pub_scara_pose(const std::vector<double>& q)
	{
		// Compute FK of the scara bot
		Vector7d finalPose;
		m_scara_kinematics->fk(q, finalPose);
		geometry_msgs::Pose out_rosPose;
		Utils::eigenPose_to_rosPose(finalPose, out_rosPose);

		m_pub_scaraPose.publish(out_rosPose);
	}


	bool ROS_Logistics::srv_scara_fk(scara_gazebo::scara_fk::Request& req,
					  	  	  	  	 scara_gazebo::scara_fk::Response& res)
	{
		// Compute FK of the scara bot
		Vector7d finalPose;
		m_scara_kinematics->fk(req.q, finalPose);
		Utils::eigenPose_to_rosPose(finalPose, res.final_pose);

		return true;
	}


	bool ROS_Logistics::srv_scara_ik(scara_gazebo::scara_ik::Request& req,
					  	  	  	  	 scara_gazebo::scara_ik::Response& res)
	{
		// Compute IK of the scara bot
		Vector7d goalPose;
		Utils::rosPose_to_eigenPose(req.goal_pose, goalPose);
		m_scara_kinematics->ik(goalPose, res.q);


//			// std_msgs::Float64 is nothing but typedef of double
//			std_msgs::Float64 target_q1;
//			target_q1.data = target_q.at(0);
//			std_msgs::Float64 target_q2;
//			target_q2.data  = target_q.at(1);
//			std_msgs::Float64 target_q3;
//			target_q3.data  = target_q.at(2);
//
//			// sending commands
//			q1_pos_controller.publish(target_q1);
//			q2_pos_controller.publish(target_q2);
//			q3_pos_controller.publish(target_q3);

		return true;
	}


	bool ROS_Logistics::srv_vel_control(scara_gazebo::scara_vel_control::Request& req,
	  	  	  	 	 	 	 	 	 	scara_gazebo::scara_vel_control::Response& res)
	{
		scara_gazebo::Float64Array desired_q;
		desired_q.data = req.q;
		m_pub_velCmd.publish(desired_q);
		res.status = true;

		return true;
	}


	/* Establish necessary services, clients, publisher, listener communication */
	void ROS_Logistics::setUp_ROS_Communication()
	{
		// Keep updating joint_states for scara_fk to use whenever needed
		m_sub_jointStates = m_node->subscribe(topic_joint_states, queue_size, &ROS_Logistics::callback_joint_states, this);

		// Setup the necessary ROS services for users to use
		m_srv_FK = m_node->advertiseService(srv_scara_fk_name, &ROS_Logistics::srv_scara_fk, this);
		m_srv_IK = m_node->advertiseService(srv_scara_ik_name, &ROS_Logistics::srv_scara_ik, this);
		m_srv_VelControl = m_node->advertiseService(srv_scara_vel_control_name, &ROS_Logistics::srv_vel_control, this);


		// Publish commands to control the robot
		m_pub_velCmd = m_node->advertise<scara_gazebo::Float64Array>(topic_joint_pos_cmd, queue_size);
		m_pub_scaraPose = m_node->advertise<geometry_msgs::Pose>(topic_scara_pose, queue_size);


//			ros::ServiceServer service1 = n2.advertiseService("/scara_qd_to_twist", srv_qd_to_twist);
//			ros::ServiceServer service2 = n2.advertiseService("/scara_twist_to_qd", srv_twist_to_qd);

	}

}    	// namespace scara
