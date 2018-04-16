cmake_minimum_required(VERSION 2.8.3)
project(navigation_mode_switcher)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  geometry_msgs
  roslint
  rospy
)

catkin_python_setup()
roslint_python()


catkin_package(
 CATKIN_DEPENDS
   actionlib_msgs
   message_runtime
   geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  ros/scripts/node_launcher_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
