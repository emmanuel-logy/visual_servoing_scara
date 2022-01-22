/**
 * This module contains the useful constants
 */

#ifndef SCARA_GAZEBO_CONSTANTS_HPP
#define SCARA_GAZEBO_CONSTANTS_HPP

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <iostream>

namespace scara
{
    // Some helpful math constants
	constexpr double pi = 3.14;
    constexpr double degToRad = pi/180;
    constexpr double radToDeg = 180/pi;


    // Some helpful ros constants
    constexpr uint32_t queue_size = 1;
    constexpr double spin_rate = 10.0;		// Keep processing callback queue at 10 Hz = 1/10 sec = 100ms


    // Some global ros topics used throughout
    const std::string param_joints_control_config {"/scara/all_joints_velocity_controller"};

    const std::string param_scara_links_length {"/scara/link_lengths"};
    const std::string param_scara_links_width {"/scara/link_width"};

    const std::string topic_joint_states {"/scara/joint_states"};
    const std::string topic_joint_pos_cmd {"/scara/joints_velocity_controller/command"};
    const std::string topic_scara_pose {"/scara/pose"};
    const std::string topic_image_raw {"/scara/camera1/image_raw"};
    const std::string topic_image_processed {"/scara/image_processed"};

    const std::string srv_scara_fk_name {"/scara_fk_srv"};
    const std::string srv_scara_ik_name {"/scara_ik_srv"};
    const std::string srv_scara_vel_control_name {"/scara_vel_control_srv"};


}		// namespace scara

#endif	// #ifndef SCARA_GAZEBO_CONSTANTS_HPP
