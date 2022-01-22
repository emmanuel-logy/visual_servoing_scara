/**
 * This module contains the class computing the Kinematics of the Scara Robot
 */

#include "ScaraConstants.hpp"
#include "scara_gazebo/Float64Array.h"
#include <ros/ros.h>
#include <ros/param.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "scara_gazebo/scara_qd_to_twist.h"
#include "scara_gazebo/scara_twist_to_qd.h"
#include "scara_gazebo/scara_ik.h"
#include "scara_gazebo/Float64Array.h"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <vector>
#include <map>
#include <iostream>
using namespace ros;
using namespace std;
using namespace Eigen;


// Some helpful constants
const double pi = 3.14;
const double degToRad = pi/180;
const double radToDeg = 180/pi;

// Robot Definition
// double L1 = 0.5;
// double L2 = 0.25;
// double L3 = 0.35;
// double W = 0.1;
// double axel_offset = 0.05;

double L1 {};
double L2 {};
double L3 {};
double W {};
double axel_offset {};

ros::Publisher pub;
double q1={}, q2={}, q3={};

ros::Publisher q1_pos_controller;
ros::Publisher q2_pos_controller;
ros::Publisher q3_pos_controller;

/*
theta -> angle in radians
d     -> distance in m
a     -> distance in m
alpha -> angle in radians
*/
Matrix4d tdh(double theta, double d, double a, double alpha)
{
  Matrix4d dh_matrix;     // typedef Matrix<double, 4, 4> Matrix4d;
  dh_matrix << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
               sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
               0,           sin(alpha),             cos(alpha),            d,
               0,           0,                      0,                     1;
  return dh_matrix;
}

/*
q1  -> in rad
q2  -> in rad
q3  -> in m
*/
void link_homogeneous_matrices(const float& q1, const float& q2, const float& q3, vector<Matrix4d>& ht)
{
    // Matrix4d A1 = tdh(q1, l1, l2, l1_offset);
    // Matrix4d A2 = tdh(q2, 180, 0, l2);
    // Matrix4d A3 = tdh(0, 0, q3, 0);
    Matrix4d A1 = tdh(q1,                   L1+W+axel_offset,    L2-axel_offset*2,   0);
    Matrix4d A2 = tdh(q2+(180*degToRad),    0,                  -L3,                180*degToRad);
    Matrix4d A3 = tdh(0,                    -axel_offset+q3,     0,                 0);


    ht.clear();
    ht.push_back(A1);
    ht.push_back(A2);
    ht.push_back(A3);
}


void getPose(const Matrix4d& ht)
{
  Eigen::Matrix3d rotation_mat = ht.block<3,3>(0,0);    // Extract a block of matrix of size 3*3 from index [0,0] of T
  Eigen::Quaterniond quat(rotation_mat);

  geometry_msgs::Pose retVal;
  retVal.position.x = ht(0,3);
  retVal.position.y = ht(1,3);
  retVal.position.z = ht(2,3);
  retVal.orientation.x = quat.x();
  retVal.orientation.y = quat.y();
  retVal.orientation.z = quat.z();
  retVal.orientation.w = quat.w();

  cout << retVal << endl;
}


void forward_kinematics_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    q1 = msg->position[0];
    q2 = msg->position[1];
    q3 = msg->position[2];

    vector<Matrix4d> ht;
    link_homogeneous_matrices(q1, q2, q3, ht);
    Matrix4d T = ht[0] * ht[1] * ht[2];

    static int i = 0;
    if (i==0)
    {
        for (int i=0; i<=2; ++i)
        {
            cout << "***********" << i << endl;
            getPose(ht[i]);
        }
        ++i;
    }

    Quaterniond quats(T.block<3,3>(0,0));
    geometry_msgs::Pose send_data;

    send_data.position.x = T(0,3);
    send_data.position.y = T(1,3);
    send_data.position.z = T(2,3);

    send_data.orientation.x = quats.x();
    send_data.orientation.y = quats.y();
    send_data.orientation.z = quats.z();
    send_data.orientation.w = quats.w();

    // cout << "POSE: \n" << send_data << endl;

    pub.publish(send_data);
}


void jacobian(Eigen::Matrix<float, 6, 3>& J)
{
    vector<Matrix4d> ht;
    link_homogeneous_matrices(q1, q2, q3, ht);

    J = Eigen::Matrix<float, 6, 3>::Zero();
    J(0,0) = ht[0](0,2);    J(1,0) = ht[0](1,2);    J(2,0) = ht[0](2,2);
    J(0,1) = ht[1](0,2);    J(1,1) = ht[1](1,2);    J(2,1) = ht[1](2,2);
    J(0,2) = ht[2](0,2);    J(1,2) = ht[2](1,2);    J(2,2) = ht[2](2,2);

    // cout << "A1" << ht[0] << endl;
    // cout << "A2" << ht[1] << endl;
    // cout << "A3" << ht[2] << endl;
    // cout << "***" << endl;
    // cout << J << endl;
}


bool srv_qd_to_twist(scara_gazebo::scara_qd_to_twist::Request  &req,
                     scara_gazebo::scara_qd_to_twist::Response &res)
{
    Eigen::Matrix<float,3,1> qd = Eigen::Matrix<float,3,1>::Zero();
    qd[0] = req.in_qd.velocity[0];
    qd[1] = req.in_qd.velocity[1];
    qd[2] = req.in_qd.velocity[2];

    Eigen::Matrix<float, 6, 3> J;
    jacobian(J);

    Eigen::Matrix<float,6,1> twist = J * qd;
    res.out_twist.linear.x = twist(0);
    res.out_twist.linear.y = twist(1);
    res.out_twist.linear.z = twist(2);
    res.out_twist.angular.x = twist(3);
    res.out_twist.angular.y = twist(4);
    res.out_twist.angular.z = twist(5);

    return true;
}

bool srv_twist_to_qd(scara_gazebo::scara_twist_to_qd::Request  &req,
                     scara_gazebo::scara_twist_to_qd::Response &res)
{
    Eigen::Matrix<float,6,1> twist = Eigen::Matrix<float,6,1>::Zero();
    twist[0] = req.in_twist.linear.x;
    twist[1] = req.in_twist.linear.y;
    twist[2] = req.in_twist.linear.z;
    twist[3] = req.in_twist.angular.x;
    twist[4] = req.in_twist.angular.y;
    twist[5] = req.in_twist.angular.z;

    Eigen::Matrix<float, 6, 3> J;
    jacobian(J);
    Eigen::Matrix<float, 3, 6> J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
    // Eigen::Matrix<float, 3, 6>a J_pinv = J.inverse();

    Eigen::Matrix<float,3,1> qd = Eigen::Matrix<float,3,1>::Zero();
    qd = J_pinv * twist;
    res.out_qd1 = qd(0);
    res.out_qd2 = qd(1);
    res.out_qd3 = qd(2);

    return true;
}


void get_robot_config(const ros::NodeHandle& n)
{
    /*
    // Example of getting using maps
    std::map<std::string,double> maps;
    ros::param::get("/scara/link_lengths", maps);
    L1 = maps["L1"];
    */

    ros::param::get("/scara/link_lengths/L1", L1);
    ros::param::get("/scara/link_lengths/L2", L2);
    ros::param::get("/scara/link_lengths/L3", L3);
    ros::param::get("/scara/link_width", W);
    axel_offset = W/2;
}

void print_pose_using_tf()
{
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	while (1)
	{
		try
		{
			geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("world", "link3", ros::Time::now(), ros::Duration(3.0));

//			geometry_msgs::Pose pose;
//			pose.orientation = transformStamped.transform.rotation;
//			pose.position = transformStamped.transform.translation;

			cout << "/////transStamped: " << transformStamped << endl;
			break;
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("** %s",ex.what());
			ros::Duration(1.0).sleep();
		}
		ros::Duration(1.0).sleep();
	}
}


void scara_ik(const geometry_msgs::Pose& target_pose, vector<double>& q)
{
	q.clear();

    // Goal pose of the end effector
    double xe = target_pose.position.x;
    double ye = target_pose.position.y;
    double ze = target_pose.position.z;

    // Making corrections with axel_offset information
    double L2_tmp = L2 - 2*axel_offset;

    double D = ( (xe*xe) + (ye*ye) - (L2_tmp*L2_tmp) - (L3*L3) ) / (2*L2_tmp*L3);
    double q2 = atan2(sqrt(1-(D*D)), D);
    static double q2_tmp = q2;

    double q1 = atan2(ye, xe) - atan2(L3*sin(q2), L2_tmp+(L3*cos(q2)));
    static double q1_tmp = q1;

    double q3 = (L1 + 2*W) - ze;

    q.push_back(q1);
    q.push_back(q2);
    q.push_back(q3);

    /* 	The inverse kinematics mapping is typically one to many. There are usually multiple sets of
    	joint variables that will yield a particular Cartesian configuration.
		Therefore, we should not verify with one particular expected numerical value.
	*/

    vector<Matrix4d> ht;
	link_homogeneous_matrices(q.at(0), q.at(1), q.at(2), ht);
	Matrix4d T = ht[0] * ht[1] * ht[2];

	Quaterniond quats(T.block<3,3>(0,0));
	geometry_msgs::Pose send_data;

	send_data.position.x = T(0,3);
	send_data.position.y = T(1,3);
	send_data.position.z = T(2,3);

	send_data.orientation.x = quats.x();
	send_data.orientation.y = quats.y();
	send_data.orientation.z = quats.z();
	send_data.orientation.w = quats.w();

	cout << "#######" << send_data << endl;
}


bool srv_scara_ik(scara_gazebo::scara_ik::Request& req,
                  scara_gazebo::scara_ik::Response& res)
{
    // Compute IK of the scara bot
	vector<double> target_q = {};
	scara_ik(req.goal_pose, target_q);

	// std_msgs::Float64 is nothing but typedef of double
	std_msgs::Float64 target_q1;
	target_q1.data = target_q.at(0);
	std_msgs::Float64 target_q2;
	target_q2.data  = target_q.at(1);
	std_msgs::Float64 target_q3;
	target_q3.data  = target_q.at(2);

	// sending commands
	q1_pos_controller.publish(target_q1);
	q2_pos_controller.publish(target_q2);
	q3_pos_controller.publish(target_q3);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scara_FK");
    ros::NodeHandle n1;

    // Load robot config from parameter server
    get_robot_config(n1);

    ros::Subscriber sub = n1.subscribe("/scara/joint_states", 1, forward_kinematics_callback);
    pub = n1.advertise<geometry_msgs::Pose>("/scara_pose", 5);
    ros::ServiceServer service = n1.advertiseService("/scara_ik_srv", srv_scara_ik);


    q1_pos_controller = n1.advertise<scara_gazebo::Float64Array>(joint_pos_cmd_topic, 5);
//    q2_pos_controller = n1.advertise<std_msgs::Float64>("/scara/joint2_position_controller/command", 5);
//    q3_pos_controller = n1.advertise<std_msgs::Float64>("/scara/joint3_position_controller/command", 5);



    ros::NodeHandle n2;
    ros::ServiceServer service1 = n2.advertiseService("/scara_qd_to_twist", srv_qd_to_twist);
    ros::ServiceServer service2 = n2.advertiseService("/scara_twist_to_qd", srv_twist_to_qd);

    ROS_INFO("scara_gazebo - FK, Jacobian");
//    ros::spin();

	ros::Rate rate(1.0);
    while (n1.ok())
    {
//    	print_pose_using_tf();

    	ros::spinOnce();
    	rate.sleep();
    }

    return 0;

}
