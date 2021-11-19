cmake_minimum_required(VERSION 3.0.2)
project(scara_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
    geometry_msgs
)

add_service_files(
  FILES
  scara_ik.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS scara_ros/include
    LIBRARIES scara_gazebo
    CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs message_runtime
    DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(scara_ik scara_ros/src/scara_ik.cpp)
target_link_libraries(scara_ik ${catkin_LIBRARIES})
