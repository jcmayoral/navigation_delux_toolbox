cmake_minimum_required(VERSION 2.8.3)
project(ros_testing_sm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  smach_ros
  audio_common_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    smach_ros
    audio_common_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
