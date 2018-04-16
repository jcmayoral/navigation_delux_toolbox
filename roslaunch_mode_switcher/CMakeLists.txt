cmake_minimum_required(VERSION 2.8.3)
project(roslaunch_mode_switcher)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  std_msgs
  roslint
  rospy
)

catkin_python_setup()
roslint_python()

add_service_files(
  FILES
  ModeSwitcher.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
 CATKIN_DEPENDS
   actionlib_msgs
   message_runtime
   std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

install(PROGRAMS
  ros/scripts/node_launcher_test
  ros/scripts/mode_manager_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
