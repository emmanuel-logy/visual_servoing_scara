cmake_minimum_required(VERSION 3.0.2)
project(scara-gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
)

catkin_package(
    INCLUDE_DIRS scara_ros/include
    LIBRARIES scara_gazebo
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
    DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(talker scara_ros/src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener scara_ros/src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

