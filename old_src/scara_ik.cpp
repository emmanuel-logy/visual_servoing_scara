#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "scara_gazebo/scara_ik.h"

bool scara_ik_srv(scara_gazebo::scara_ik::Request& req,
                  scara_gazebo::scara_ik::Response& res)
{
    /* Compute IK of the scara bot */
    // Can get robot diemensions
    double axel_offset = 0.05;
    double L1 = 0.5;
    double L2 = 0.25;
    double L3 = 0.35;
    double L4 = 0.25;

    // Goal pose of the end effector
    double xe = req.goal_pose.position.x;
    double ye = req.goal_pose.position.y;
    double ze = req.goal_pose.position.z;

    float D = ( (xe*xe) + (ye*ye) - (L2*L2) - (L3*L3) ) / (2*L2*L3);
    float q2 = atan2(sqrt(1-(D*D)), D);

    float q1 = atan2(ye, xe) - atan2(L3*sin(q2), L2+L3*cos(q2));

    float q3 = ze + axel_offset;

    std::vector<double> a {q1, q2, q3};
    res.q_res = a;

    ROS_INFO("%f %f %f", q1, q2, q3);

    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scara_ik_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("scara_ik_srv", scara_ik_srv);
  ROS_INFO("Ready to compute IK of scara bot");
  ros::spin();

  return 0;
}
