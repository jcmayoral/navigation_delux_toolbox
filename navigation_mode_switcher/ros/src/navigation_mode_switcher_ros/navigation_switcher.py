#!/usr/bin/python
import roslaunch
import rospy
import rospkg
import os


class NavigationMode:
    def __init__(self, package = None, launch_file = None):
        rospy.init_node('en_Mapping', anonymous=True)
        #rospy.on_shutdown(self.shutdown)
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/jose/ros_ws/src/mas_domestic_robotics/mdr_navigation/mdr_2dslam/ros/launch/2dslam.launch"])

    def start(self):
        self.launch.start()

    def stop(self):
        self.launch.shutdown()
