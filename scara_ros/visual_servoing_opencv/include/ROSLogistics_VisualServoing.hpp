/**
 * This module contains the class managing the ROS Logistics
 */

#ifndef ROS_LOGISTICS_VISUAL_SERVOING_HPP
#define ROS_LOGISTICS_VISUAL_SERVOING_HPP

#include "Utils.hpp"
#include "ScaraConstants.hpp"
#include "VisualServoing.hpp"
#include "scara_gazebo/scara_ik.h"
#include "scara_gazebo/scara_fk.h"
#include "scara_gazebo/scara_vel_control.h"
#include <sensor_msgs/JointState.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <memory>

namespace scara
{
	class VisualServoing;	// Forward Declaration

	class ROSLogistics_VisualServoing
	{
	public:
		ROSLogistics_VisualServoing();

		~ROSLogistics_VisualServoing() = default;

		void callback_joint_states(const sensor_msgs::JointState::ConstPtr& msg);

		void callback_raw_image(const sensor_msgs::ImageConstPtr& msg_img,
								const sensor_msgs::CameraInfoConstPtr& msg_camInfo);

		// called by ros::Timer
		void callback_timer(const ros::TimerEvent&);

		// called by m_visual_servo to restart one-shot timer when target is locked
		void reset_timer();

		// For rosbag cmdline to record the processed image
		void pub_processedImg(const cv::Mat& cv_img);

		// For Visual Servoing to talk to other ROS Nodes
		void call_fk_srv(const std::vector<double>& q, Vector7d& pose);

		void call_ik_srv(const Vector7d& pose, std::vector<double>& q);

		void call_velCmd_srv(const std::vector<double>& q);



	private:
        std::shared_ptr<ros::NodeHandle> m_node;	// ptr cuz we shouldn't create it before calling ros::init

        std::shared_ptr<VisualServoing> m_visual_servo;

		ros::Subscriber m_sub_jointStates;

		ros::ServiceClient m_client_FK;

		ros::ServiceClient m_client_IK;

		ros::ServiceClient m_client_VelControl;

		ros::Publisher m_pub_processedImg;

		ros::Timer m_stablize_wait_timer;

		double m_stablize_wait_time;


		/* Image Processing Stuff */
		std::shared_ptr<image_transport::ImageTransport> m_img_transport;

		// image_transport::Subscriber m_img_sub;			// when subscribing to only raw_image
		image_transport::CameraSubscriber m_img_sub;

		// To get camera information like resolution, etc
		image_geometry::PinholeCameraModel m_cam_model;

		/* Establish necessary services, clients, publisher, listener communication */
		void setUp_ROS_Communication();
	};


}    	// namespace scara

#endif	// #ifndef ROS_LOGISTICS_VISUAL_SERVOING_HPP
