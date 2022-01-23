/**
 * This module contains the class handling the PID controller computation
 */

#ifndef MY_VELOCITY_CONTROLLER_DEFINES_HPP
#define MY_VELOCITY_CONTROLLER_DEFINES_HPP

#include "ScaraConstants.hpp"
#include "visual_servoing_scara/Float64Array.h"
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <deque>

namespace scara
{
	class MyVelocityController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
	{

	public:

		/*
		 * [1] 	To cache all the handles needed to invoke later
		 * 		Eg: Handle to issue command to joint in Gazebo
		 * [2] 	To load all the necessary parameters
		 * 		Eg: PID values
		 * [3] 	To subscribe to necessary topics to compute control signal
		 * 		Eg: /scara/joint1_position_controller/command
		 */
		bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& n);

		/*
		 * [?]	Don't know at the moment
		 */
		void starting(const ros::Time &time);

		/*
		 * [?]	Don't know at the moment
		 */
		void stopping(const ros::Time &time);

		/*
		 * [1]	This is called continuously by the gazebo process
		 * [2]	Computation of control signal using PID algorithm happens here
		 */
		void update(const ros::Time &time, const ros::Duration &period);

		/*
		 * [1]	Custom callback function for whatever topic we subscribe to
		 * 		Basically, some main code like visual servoing tells the velocity input to be applied
		 * 		Or some IK srv tells the goal position to go and this node calculates the velocity to reach there ASAP smoothly
		 */
		void set_cmd_cb(const visual_servoing_scara::Float64Array::ConstPtr& msg);



	private:
			struct PID_Gain
			{
				PID_Gain() : Kp(0.0), Ki(0.0), Kd(0.0) {}

				double Kp;
				double Ki;
				double Kd;
			};

			std::vector<hardware_interface::JointHandle> m_joint_handles;

			std::vector<PID_Gain> m_joint_PID_Gains;

			// For performing integrator anti-windup, we calculate iMax and iMin based on DAC (max-min) range limit
			int actuator_DAC_range;					// We cannot apply a signal greater than what DAC can apply on actuator
			// For integrating over only certain batch of errors, else it keeps increasing forever and leads to osciallations
			int m_iSamplingSize;					// Let's integrate over prev 10 err values only
			std::deque<double> m_iErrAccumulator;
			double m_iErrSum;

			// For finding differential of error
			double m_prev_err;

			std::vector<double> m_joint_cmds;

			ros::Subscriber m_sub_cmd;

			ros::NodeHandle m_node;

			void load_pid_gains();
	};

}    	// namespace scara

#endif	// #ifndef MY_VELOCITY_CONTROLLER_DEFINES_HPP

