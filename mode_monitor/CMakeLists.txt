cmake_minimum_required(VERSION 2.8.3)
project(mode_monitor)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  std_msgs
  roslint
  roscpp
  costmap_2d
  nav_msgs
  tf
  roslaunch_mode_switcher
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
 CATKIN_DEPENDS
   actionlib_msgs
   std_msgs
   costmap_2d
   tf
   nav_msgs
   roslaunch_mode_switcher
)

add_library(mode_monitor_ros
  ros/src/mode_monitor.cpp
)

add_executable(mode_monitor_node
  ros/src/mode_monitor_node.cpp
)

target_link_libraries(mode_monitor_node mode_monitor_ros ${catkin_LIBRARIES})

install(TARGETS
  mode_monitor_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
  mode_monitor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
