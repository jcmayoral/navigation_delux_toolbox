#! /usr/bin/python
import rospy
import yaml
from roslaunch_mode_switcher_ros.mode_switcher import RosLaunchMode
from roslaunch_mode_switcher.srv import *

class ModeManager:
    def __init__(self,config_file,default):
        if rospy.has_param('~config_file'):
            config_file = rospy.get_param("~config_file")
        else:
            rospy.logerr("You must specify a definition_file")
            return
        stream = file(config_file, 'r')
        self.modes = yaml.load(stream)
        self.available_modes = dict()
        stream.close()

        self.current_mode = None
        self.is_stop_requested = False
        self.request_mode = rospy.Service('request_mode', ModeSwitcher, self.runService)

        self.find_modes()
        self.execute_mode(default)

    def find_modes(self):
        for key, value in self.modes.iteritems():
            self.available_modes[key] = value

    def execute_mode(self,mode):
        self.current_mode = RosLaunchMode(package = self.available_modes[mode]['package'],
                                          route_to_launch_file = self.available_modes[mode]['path_to_file'])
        rospy.sleep(1.0)
        return self.current_mode.start()

    def runService(self,req):

        resp = ModeSwitcherResponse()
        #Close all
        if req.request_mode.data == "Stop":
            self.is_stop_requested = True
            resp.succeeded.data = True
            return resp

        self.current_mode.stop()
        rospy.logerr("Previous mode has stopped")
        self.current_mode = None
        rospy.sleep(2.0)

        rospy.logwarn("Current mode is starting")
        resp.succeeded.data = self.execute_mode(req.request_mode.data)
        resp.succeeded.data = True

        return resp
