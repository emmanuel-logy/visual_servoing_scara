/**
 * This module contains the class handling the PID controller computation
 */

#include "MyVelocityController.hpp"
#include <map>

namespace scara
{
	bool MyVelocityController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& n)
	{
		m_node = n;

		// Get all joint names and their corresponding handles3

		// [1] Get one joint's name
		std::vector<std::string> joint_names;
		if (!m_node.getParam(param_joints_control_config+"/joints", joint_names))
		{
			ROS_ERROR("Could not find joints name)");
			return false;
		}

		int count = 1;
		for (auto& i: joint_names)
		{
			// [2] Get this joint's corresponding handle
			// If getHandle fails, it throws an exception and let it fail
			m_joint_handles.push_back(hw->getHandle(i));

			// [3] Get PID values for every joint
			load_pid_gains();
		}

		// Set initial positions to home config i.e. all q=0.0
		m_joint_cmds = {0.0, 0.0, 0.0};

		// Subscribe to commands
		m_sub_cmd = m_node.subscribe(topic_joint_pos_cmd, queue_size, &MyVelocityController::set_cmd_cb, this);

		// For Derivative signal
		m_prev_err = 0;		// Let's start assuming zero error

		// For Intergral signal
		actuator_DAC_range = 255;
		m_iErrSum = 0;
		m_iSamplingSize = 3;
		m_iErrAccumulator.resize(m_iSamplingSize, 0.0);

		return true;
	}


	void MyVelocityController::update(const ros::Time &time, const ros::Duration &period)
	{
		// Generate PID control signals for all the joints
		for (int i=0; i<m_joint_handles.size(); ++i)
		{
			// [1] Find error
			double desired_q = m_joint_cmds[i];
			double current_q = m_joint_handles[i].getPosition();
			double error = desired_q - current_q;


			// [2] Generate Porportional control signal
			double P = m_joint_PID_Gains[i].Kp * error;


			// [3] Generate Integral control signal
			// Mul with 0.1 to limit the contribution from Integrator control signal part because final control signal is combo of P+I+D
//			double iMax = 0.1 * (actuator_DAC_range)/m_joint_PID_Gains[i].Ki;
//			double iMin = -0.1 * (actuator_DAC_range)/m_joint_PID_Gains[i].Ki;
			double iMax = 0.1 * (actuator_DAC_range);
			double iMin = -0.1 * (actuator_DAC_range);

			m_iErrSum = m_iErrSum + error + m_iErrAccumulator.front();
			m_iErrAccumulator.pop_front();								// throw away oldest error within the sampling size batch
			m_iErrAccumulator.push_back(error);
			if 		(m_iErrSum>iMax)
				m_iErrSum = iMax;
			else if (m_iErrSum<iMin)
				m_iErrSum = iMin;
			double I = m_joint_PID_Gains[i].Ki * m_iErrSum;
			// Another logic that can be added is turning ON integrator only when almost reaching the target point, i.e. when err reduces below certain threshold


			// [4] Generate Derivative control signal
			double D = m_joint_PID_Gains[i].Kd * (error - m_prev_err);
			m_prev_err = error;


			// [5] Summing all the above, generate the final control signal
			double vel_cmd = P + I + D;


			// [6] Apply the control signal to the each joint's actuator
			m_joint_handles[i].setCommand(vel_cmd);
		}
	}


	void MyVelocityController::set_cmd_cb(const visual_servoing_scara::Float64Array::ConstPtr& msg)
	{
		m_joint_cmds = msg->data;

		// Load any new pid gains from param server if updated from cmdline for tuning purposes
		load_pid_gains();
	}


	void MyVelocityController::starting(const ros::Time &time) { }


	void MyVelocityController::stopping(const ros::Time &time) { }


	void MyVelocityController::load_pid_gains()
	{
		m_joint_PID_Gains.clear();

		for (int count=1; count<=m_joint_handles.size(); ++count)
		{
			std::map<std::string,double> pid_config_map;
			if (!ros::param::get(param_joints_control_config+"/pid/joint"+std::to_string(count), pid_config_map))
			{
				ROS_ERROR("Could not find pid config for joint");
				return;
			}
			PID_Gain tmp;
			tmp.Kp = pid_config_map["p"];	// not doing error handling for now
			tmp.Ki = pid_config_map["i"];
			tmp.Kd = pid_config_map["d"];
			m_joint_PID_Gains.push_back(tmp);
		}
	}


	PLUGINLIB_EXPORT_CLASS(scara::MyVelocityController, controller_interface::ControllerBase);

} // namespace scara


