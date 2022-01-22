/**
 * This module contains the class position-based visual servoing (PBVS) for the Scara Robot
 */

#ifndef VISUAL_SERVOING_HPP
#define VISUAL_SERVOING_HPP

#include "ROSLogistics_VisualServoing.hpp"
#include "ScaraConstants.hpp"
#include <opencv2/highgui/highgui.hpp>


namespace scara
{
	struct CameraInfo
	{
		Eigen::Vector2d pixel_resolution;
	};

	class ROSLogistics_VisualServoing;	// Forward Declaration

	class VisualServoing
	{
	public:
		VisualServoing(const std::shared_ptr<ROSLogistics_VisualServoing> rosLogistics);

		~VisualServoing();

		// ROSLogistics_VisualServoing updates q values required for Visual Servoing
		void update_joint_states(const std::vector<double>& q);

		void update_camera_info(const CameraInfo& camInfo);

		// Hit the target and retract once robot has stablized
		void update_timer_callback();

		void set_ref_feature();

		void process_raw_image(const cv::Mat& cv_ptr);

	private:

		// To perform visual servoing
		std::vector<double> m_joints_state;

		// Output of perform_PBVS()
		std::vector<double> m_desired_q;

		bool m_hit_target;

        std::shared_ptr<ROSLogistics_VisualServoing> m_ROS_Logistics;

        CameraInfo m_camInfo;

		// Image Processing related stuff
		cv::Mat m_current_img;

		cv::Point m_currentCenter_in_pixelFrame;

		cv::Point m_refCenter_in_pixelFrame;


		// Find circle center using the Hough Transform from opencv
		void find_circle_center();

		// Perform Position Based Visual Servoing
		void perform_PBVS();
	};

}    	// namespace scara

#endif	// #ifndef VISUAL_SERVOING_HPP
