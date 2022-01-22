/**
 * This module contains the class managing the ROS Logistics
 */
#include "ROSLogistics_VisualServoing.hpp"
#include "ScaraConstants.hpp"
#include "ScaraKinematics.hpp"
#include "scara_gazebo/Float64Array.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
//#include "scara_gazebo/scara_qd_to_twist.h"
//#include "scara_gazebo/scara_twist_to_qd.h"


namespace scara
{

	ROSLogistics_VisualServoing::ROSLogistics_VisualServoing()	:	m_node(nullptr)
	{
		// Register the node with ROS Master
		m_node = std::make_shared<ros::NodeHandle>();

		m_img_transport = std::make_shared<image_transport::ImageTransport>(*m_node);

		/* When the image processing is changed to PCL, just change the object name here and
		 * the rest of the code needn't be touched */
		m_visual_servo = std::make_shared<scara::VisualServoing>(std::shared_ptr<ROSLogistics_VisualServoing>(this));

		// Establish all necessary connections
		setUp_ROS_Communication();
		ROS_INFO("scara_visual_servoing_opencv all clients and listeners up and running . . .");


		// Subscribe to a timer to wait till scara stablizes and then hit the target and retract
		double stablize_wait_time = 0;	// initializing
		m_stablize_wait_timer = m_node->createTimer(ros::Duration(stablize_wait_time),
												    &ROSLogistics_VisualServoing::callback_timer, this,
												    true);	// true == one-shot timer
		m_stablize_wait_timer.stop();	// Don't need now.. once stablized, we'll trigger later

		// Keep looking into service, subscriber queues once in 100ms
		ros::Rate rate(spin_rate);	// 10 Hz = 1/10 sec = 100ms
		while (m_node->ok())
		{
			ros::spinOnce();
			rate.sleep();
		}
	}


	void ROSLogistics_VisualServoing::callback_joint_states(const sensor_msgs::JointState::ConstPtr& msg)
	{
		m_visual_servo->update_joint_states(msg->position);
	}


	void ROSLogistics_VisualServoing::callback_raw_image(const sensor_msgs::ImageConstPtr& msg_img,
										   	   	   	   	 const sensor_msgs::CameraInfoConstPtr& msg_camInfo)
	{
        // No need to keep updating in every loop
        static bool runOnce = true;
        while(runOnce)
        {
            // [1] Get camera info
			m_cam_model.fromCameraInfo(msg_camInfo);
			scara::CameraInfo camInfo;
			camInfo.pixel_resolution << m_cam_model.fullResolution().width,
									    m_cam_model.fullResolution().height;
			m_visual_servo->update_camera_info(camInfo);


			// [2] Set reference feature to track, i.e. center of circle in our case
			m_visual_servo->set_ref_feature();

            runOnce = false;
        }


		// [3] Convert sensor_image to BGR cv_image. Remove ROS dependency before sending to Visual Servoing object
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
        	cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("CV Bridge Exception: %s", e.what());
            return;
        }

       // [4] keep processing image to track the object
        m_visual_servo->process_raw_image(cv_ptr->image);
	}


	void ROSLogistics_VisualServoing::callback_timer(const ros::TimerEvent&)
	{
		m_visual_servo->update_timer_callback();
	}


	void ROSLogistics_VisualServoing::reset_timer(const double& time_period)
	{
		m_stablize_wait_timer.stop();
		m_stablize_wait_timer.setPeriod(ros::Duration(time_period));
		m_stablize_wait_timer.start();
	}


	void ROSLogistics_VisualServoing::pub_processedImg(const cv::Mat& cv_img)
	{
		cv_bridge::CvImage img_bridge;
		sensor_msgs::Image rosSensor_img; 			// message to be sent

		static uint32_t counter = 0;
		++counter;
		std_msgs::Header header; 				// empty header
		header.seq = counter; 					// user defined counter
		header.stamp = ros::Time::now(); 		// time
		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, cv_img);
		img_bridge.toImageMsg(rosSensor_img); // from cv_bridge to sensor_msgs::Image
		m_pub_processedImg.publish(rosSensor_img);
	}


	void ROSLogistics_VisualServoing::call_fk_srv(const std::vector<double>& q, Vector7d& pose)
	{
		if (m_client_FK)
		{
			scara_gazebo::scara_fk srv;
			srv.request.q = q;
			if ( !m_client_FK.call(srv) )
			{
				ROS_ERROR("scara_fk service call failed");
			}
			Utils::rosPose_to_eigenPose(srv.response.final_pose, pose);
		}
		else
		{
			ROS_ERROR("scara_fk service persistent client handle lost :(");
		}
	}


	void ROSLogistics_VisualServoing::call_ik_srv(const Vector7d& pose, std::vector<double>& q)
	{
		if (m_client_IK)
		{
			scara_gazebo::scara_ik srv;
			Utils::eigenPose_to_rosPose(pose, srv.request.goal_pose);
			if ( !m_client_IK.call(srv) )
			{
				ROS_ERROR("scara_ik service call failed");
			}
			q = srv.response.q;
		}
		else
		{
			ROS_ERROR("scara_fk service persistent client handle lost :(");
		}
	}


	void ROSLogistics_VisualServoing::call_velCmd_srv(const std::vector<double>& q)
	{
		if (m_client_VelControl)
		{
			scara_gazebo::scara_vel_control srv;
			srv.request.q = q;
			if ( !m_client_VelControl.call(srv) )
			{
				ROS_ERROR("scara_ik service call failed");
			}
		}
		else
		{
			ROS_ERROR("scara_fk service persistent client handle lost :(");
		}
	}


	void ROSLogistics_VisualServoing::setUp_ROS_Communication()
	{
		// Keep updating joint_states for scara_fk to use whenever needed
		m_sub_jointStates = m_node->subscribe(topic_joint_states, queue_size, &ROSLogistics_VisualServoing::callback_joint_states, this);

		// Setup the necessary ROS services for computing FK, IK and VEL_CNTRL to perform visual servoing
		m_client_FK 		= m_node->serviceClient<scara_gazebo::scara_fk>(srv_scara_fk_name, true);					// 2nd arg true==>persistent connection
		m_client_IK 		= m_node->serviceClient<scara_gazebo::scara_ik>(srv_scara_ik_name, true);					// 2nd arg true==>persistent connection
		m_client_VelControl = m_node->serviceClient<scara_gazebo::scara_vel_control>(srv_scara_vel_control_name, true);	// 2nd arg true==>persistent connection


		// Setup the necessary ROS subscriptions to listen to raw_image and camera info
		// To subscribe to raw_image alone
		// m_img_sub = m_img_transport.subscribe("/scara/camera1/image_raw", 1, &VisualServoing::callback_raw_image, this);
		m_img_sub = m_img_transport->subscribeCamera(topic_image_raw, 1, &ROSLogistics_VisualServoing::callback_raw_image, this);

		m_pub_processedImg = m_node->advertise<sensor_msgs::Image>(topic_image_processed, queue_size);
	}

}    	// namespace scara
