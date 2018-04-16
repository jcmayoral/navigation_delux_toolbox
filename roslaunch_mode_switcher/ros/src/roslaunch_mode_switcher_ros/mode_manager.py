#! /usr/bin/python
import rospy
import yaml
from roslaunch_mode_switcher_ros.mode_switcher import RosLaunchMode
from roslaunch_mode_switcher.srv import ModeSwitcher

class ModeManager:
    def __init__(self,config_file):
        if rospy.has_param('~config_file'):
            config_file = rospy.get_param("~config_file")
        else:
            rospy.logerr("You must specify a definition_file")
            return
        stream = file(config_file, 'r')
        modes = yaml.load(stream)
        self.available_modes = dict()
        stream.close()

        self.current_mode = None
        self.is_stop_requested = False

        for key, value in modes.iteritems():
            print type(key), key
            self.available_modes[key] = RosLaunchMode(package = value['package'],
                                                      route_to_launch_file = value['path_to_file'])

        self.request_mode = rospy.Service('request_mode', ModeSwitcher, self.runService)


    def runService(self,req):

        if req.request_mode is 'Stop':
            self.is_stop_requested = True

        if self.current_mode is not None:
            self.available_modes[self.current_mode].stop()

        print req.request_mode
        self.available_modes[req.request_mode.data].start()

        self.current_mode = req.request_mode.data
