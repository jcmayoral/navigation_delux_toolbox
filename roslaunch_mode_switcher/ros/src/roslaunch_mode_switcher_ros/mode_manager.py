#! /usr/bin/python
import rospy
import yaml
from roslaunch_mode_switcher_ros.mode_switcher import RosLaunchMode

class ModeManager:
    def __init__(self,config_file):
        if rospy.has_param('~config_file'):
            config_file = rospy.get_param("~config_file")
        else:
            rospy.logerr("You must specify a definition_file")
            return
        stream = file(config_file, 'r')
        modes = yaml.load(stream)
        stream.close()

        for key, value in modes.iteritems():
            print key
            RosLaunchMode(package = value['package'], route_to_launch_file = value['path_to_file'])

        print type(modes)
