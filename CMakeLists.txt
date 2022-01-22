cmake_minimum_required(VERSION 3.0.2)
project(scara_gazebo)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Debug")		# change it to "Release" when needed

################################################
## Find all ROS dependencies ##
################################################

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    tf2_ros
    controller_interface
    hardware_interface
    pluginlib
  	cv_bridge
  	image_transport
  	image_geometry
)

find_package(OpenCV 4 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  scara_fk.srv
  scara_ik.srv
  scara_vel_control.srv
  scara_qd_to_twist.srv
  scara_twist_to_qd.srv
)

add_message_files(
  FILES
  Float64Array.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs	
)

###################################
## Catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS scara_ros/include
    LIBRARIES scara_gazebo
    CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs message_runtime 
    DEPENDS
)

###########
## Build ##
###########

include_directories(
	scara_ros/include
	scara_ros/common_utils/include
    scara_ros/common_utils/include/eigen
    scara_ros/kinematics/include
    scara_ros/plugin_myVelController/include
    scara_ros/visual_servoing_opencv/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)


########## Building common_utils.so ##########
set(SOURCES_COMMON_UTILS
		scara_ros/common_utils/src/Utils.cpp
)
add_library(common_utils ${SOURCES_COMMON_UTILS})
target_link_libraries(common_utils ${catkin_LIBRARIES})


########## Building kinematics node exe ##########
#file(GLOB SOURCES "scara_ros/src/*.cpp")
set(SOURCES_KINEMATICS 
		scara_ros/kinematics/src/main.cpp
		scara_ros/kinematics/src/ROSLogistics.cpp
		scara_ros/kinematics/src/ScaraKinematics.cpp	
)
add_executable(scara_kinematics ${SOURCES_KINEMATICS})
add_dependencies(scara_kinematics scara_gazebo_generate_messages_cpp)
target_link_libraries(scara_kinematics ${catkin_LIBRARIES} common_utils)


########## Building my_vel_controller plugin .so ##########
set(SOURCES_PLUGIN_MY_VEL_CONTROLLER
		scara_ros/plugin_myVelController/src/MyVelocityController.cpp
)
add_library(my_velocity_controller ${SOURCES_PLUGIN_MY_VEL_CONTROLLER})
target_link_libraries(my_velocity_controller ${catkin_LIBRARIES} common_utils)


########## Building visual servoing node exe ##########
set(SOURCES_VISUAL_SERVOING_OPENCV
		scara_ros/visual_servoing_opencv/src/main.cpp
		scara_ros/visual_servoing_opencv/src/ROSLogistics_VisualServoing.cpp
		scara_ros/visual_servoing_opencv/src/VisualServoing.cpp	
)
add_executable(visual_servoing ${SOURCES_VISUAL_SERVOING_OPENCV})
target_link_libraries(visual_servoing ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} common_utils)


#############
## Testing ##
#############

#############
## Install ##
#############
