#!/usr/bin/python
import roslaunch
import rospy
import rospkg

class RosLaunchMode:
    def __init__(self, package = None, route_to_launch_file = None):
        rospy.loginfo("Constructor of RosLaunchMode")
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        package_path = rospkg.RosPack().get_path(package)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [package_path + "/" + route_to_launch_file])

    def start(self):
        try:
            self.launch.start()
        except Exception as e:
            print(e)
            rospy.logerr("Launch file not started")

    def stop(self):
        self.launch.shutdown()
