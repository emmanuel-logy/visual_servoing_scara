/**
 * This module contains the class position-based visual servoing (PBVS) for the Scara Robot
 */

#include "VisualServoing.hpp"
// For Hough Circle Transform
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <cmath>

namespace scara
{

	VisualServoing::VisualServoing(const std::shared_ptr<ROSLogistics_VisualServoing> rosLogistics): 	m_ROS_Logistics(rosLogistics),
																										m_refCenter_in_pixelFrame(0, 0),
																										m_hit_target(true)
	{
		cv::namedWindow("view");
	}


	VisualServoing::~VisualServoing()
	{
		cv::destroyWindow("view");
	}


	void VisualServoing::update_joint_states(const std::vector<double>& q)
	{
		m_joints_state = q;
	}


	void VisualServoing::update_camera_info(const CameraInfo& camInfo)
	{
		m_camInfo = camInfo;
	}


	void VisualServoing::update_timer_callback()
	{
<<<<<<< HEAD
		ROS_INFO("!!! Target Locked !!!");

=======

		std::cout << "Timer called" << std::endl;
>>>>>>> ea81a5977bfdfcb812a2a96024af55bb17b151fd
		m_desired_q[2] = 0.4;							// 0.35 based on table heigth for prsimatic joint to touch the object
		m_ROS_Logistics->call_velCmd_srv(m_desired_q);
		ros::Duration(0.5).sleep();						// wait for 0.5 sec before retracting the prsimatic joint
		m_desired_q[2] = 0;
		m_ROS_Logistics->call_velCmd_srv(m_desired_q);	// wait for 1.5 sec to allow retracting the prsimatic joint to home pos
		ros::Duration(1).sleep();

		m_hit_target = true;
	}


	void VisualServoing::set_ref_feature()
	{
	   /* Circle ref center is (150,150) because camera resolution is 300*300
		 * We take exactly half because our goal is to track the center of the circle at center of image pixel frame...
		 * In other words, we want the circle center and the image pixel frame center to coincide
		 * Center of the circle is the feature we've chosen
		 */
        m_refCenter_in_pixelFrame.x = m_camInfo.pixel_resolution[0]/2;
        m_refCenter_in_pixelFrame.y = m_camInfo.pixel_resolution[1]/2;
	}


	void VisualServoing::process_raw_image(const cv::Mat& cv_img)
	{
		// [1] ROSLogistics_VisualServoing class converts sensor_image to BGR cv_image
		// 	   Here, we convert BGR cv_image into cv::Mat to apply matrices linear algebra
		m_current_img = cv_img;

/*
//        // Code for frame transform from world frame to camera frame
//        m_cam_model.fromCameraInfo(msg_camInfo);
//        tf2_ros::Buffer tfBuffer;
//		tf2_ros::TransformListener tfListener(tfBuffer);
//
//		// Keep trying till a few frames are loaded
//		while (1)
//		{
//			try
//			{
//				ros::Time acquisition_time = msg_camInfo->header.stamp;
//				geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("world",
//																							m_cam_model.tfFrame(),
//																							ros::Time::now(),
//																							ros::Duration(1.0 / 30));
//				// std::cout << "*******transStamped: " << transformStamped << std::endl;
//				// std::cout << "x: " << transformStamped.transform.translation.x << " y: " << transformStamped.transform.translation.y << std::endl;
//				break;
//			}
//			catch (tf2::TransformException &ex)
//			{
//				ROS_WARN("** %s",ex.what());
//			}
//			ros::Duration(1.0).sleep();
//		}
*/

        // [2] Now find the image center and save it in m_currentCenter_in_pixelFrame
        find_circle_center();

        // [3a] Show detected circles
		cv::imshow("view", m_current_img);
		cv::waitKey(30);


		// [3b] Publish the processed image over a topic for rosbag to record it
		m_ROS_Logistics->pub_processedImg(m_current_img);


		// [4] Perform Position Based Visual Servoing
		/* # Find difference between ref and current circle center
		 * # Find the pose end effector in cartesian coordinates to make the center of circle coincide with pixel frame center
		 * # Find new desired q values (joint angles) of the scara robot using the IK service
		 * # Pass this desired q values to our velocity controller and wait for the visual servoing magic to happen ;)
		 */
		perform_PBVS();
	}


	void VisualServoing::find_circle_center()
	{
	    // Threshold with blue filter to help hough transform to detect our blue circle
//		const cv::Scalar blue_low = cv::Scalar(110,200,50);		// blue in HSV
//		const cv::Scalar blue_high = cv::Scalar(130,255,255);	// blue in HSV
//	    cv::Mat blue_filtered_hsv;
//	    cv::cvtColor(m_current_img, blue_filtered_hsv, cv::COLOR_BGR2HSV);
//	    cv::inRange(blue_filtered_hsv, blue_low, blue_high, blue_filtered_hsv);
//	    cv::imshow("view", blue_filtered_hsv);
//		cv::waitKey(30);
//		ros::Duration(1).sleep();
//
//		cv::Mat blue_filtered_bgr;
//		cv::cvtColor(blue_filtered_hsv, blue_filtered_bgr, cv::COLOR_HSV2BGR);
//	    cv::Mat gray;
//	    cv::cvtColor(blue_filtered_bgr, blue_filtered_bgr, cv::COLOR_BGR2HSV);
//	    cv::medianBlur(blue_filtered_bgr, blue_filtered_bgr, 5);

	    cv::Mat gray;
	    cv::cvtColor(m_current_img, gray, cv::COLOR_BGR2GRAY);
	    cv::medianBlur(gray, gray, 5);


//	    int in1=0, in2=0;
//	    std::cin >> in1;
//	    std::cin >> in2;
	    std::vector<cv::Vec3f> circles;
	    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
	    		gray.rows/64,  // change this value to detect circles with different distances to each other
				200, 50, 0, 0 // change the last two parameters
	            // (min_radius & max_radius) to detect larger circles
	    );

	    for( size_t i = 0; i < circles.size(); i++ )
	    {
	        cv::Vec3i c = circles[i];
	        m_currentCenter_in_pixelFrame = cv::Point(c[0], c[1]);
	        // circle center
	        cv::circle( m_current_img, m_currentCenter_in_pixelFrame, 1, cv::Scalar(0,100,100), 2, cv::LINE_AA);
	        // circle outline
	        int radius = c[2];
	        cv::circle( m_current_img, m_currentCenter_in_pixelFrame, radius, cv::Scalar(255,0,255), 2, cv::LINE_AA);
	    }

//		std::cout << "Image Center: " << m_currentCenter_in_pixelFrame << std::endl;
	}


	void VisualServoing::perform_PBVS()
	{
		/* -------------------------------------------------------------------------------------------------------------------------- */
		// [1] Find difference between ref and current circle center
		cv::Point err_center_pos = m_refCenter_in_pixelFrame - m_currentCenter_in_pixelFrame;

		/* -------------------------------------------------------------------------------------------------------------------------- */
		// [2] Find the pose end effector in cartesian coordinates to make the center of circle coincide with pixel frame center
		Vector7d current_pose;
		m_ROS_Logistics->call_fk_srv(m_joints_state, current_pose);

		// For derivation of this formula, check out the notes
		/* Convert the error from pixel frame to cartesian frame
		 * In our case, 1m in gazebo cartesian coordinate equals 300 pixels in image pixel frame
		 * Therefore, 1pixel = 1/300 m;
		 */
		double center_pos_err_x = (err_center_pos.y)/m_camInfo.pixel_resolution[0];
		double center_pos_err_y = (err_center_pos.x)/m_camInfo.pixel_resolution[1];

		double desired_pose_x = current_pose[0] + center_pos_err_x;
		double desired_pose_y = current_pose[1] + center_pos_err_y;

		Vector7d desired_pose = current_pose;
		desired_pose[0] = desired_pose_x;
		desired_pose[1] = desired_pose_y;

		/* -------------------------------------------------------------------------------------------------------------------------- */
		// [3] Find new desired q values (joint angles) of the scara robot using the IK service
		m_ROS_Logistics->call_ik_srv(desired_pose, m_desired_q);

		/* -------------------------------------------------------------------------------------------------------------------------- */
		// [4] Pass this desired q values to our velocity controller and wait for the visual servoing magic to happen ;)
		for (auto& i : m_desired_q)
		{
			if (std::isnan(i))
			{
				ROS_ERROR("Object center outside robot's task space");
				return;
			}
		}
//		std::cout << "Desired q: " << m_desired_q[0] << " " << m_desired_q[1] << " " << m_desired_q[2] << std::endl;
		m_ROS_Logistics->call_velCmd_srv(m_desired_q);

		/* -------------------------------------------------------------------------------------------------------------------------- */
		// [5] When the feature is tracked succesfully, i.e. when circle center coincides with center of pixel frame,
		//	   make the prismatic joint hit the target point
		cv::Point err_lower_bound = m_refCenter_in_pixelFrame - cv::Point(2,2);
		cv::Point err_higher_bound = m_refCenter_in_pixelFrame + cv::Point(2,2);
		if ( (m_currentCenter_in_pixelFrame.x >= err_lower_bound.x && m_currentCenter_in_pixelFrame.y >= err_lower_bound.y) ||
			 (m_currentCenter_in_pixelFrame.x <= err_higher_bound.x && m_currentCenter_in_pixelFrame.y <= err_higher_bound.y) )
		{
			// dont trigger timer when the timer has already been triggered
			if (m_hit_target)
			{
				// Hit the target point after the robot has stabilized.. To allow certain time, we use ros::Timer
				m_ROS_Logistics->reset_timer();		// start one-shot timer once target is locked
				m_hit_target = false;				// don't trigger till this flag is reset by timer callback
			}
		}
	}

}    	// namespace scara
