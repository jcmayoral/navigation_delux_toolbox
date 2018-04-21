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

        for key, value in self.modes.iteritems():
            print key
            if key == default:
                print "jere"
                self.available_modes[key] = RosLaunchMode(package = value['package'],
                                                      route_to_launch_file = value['path_to_file'])
        self.current_mode = default
        self.is_stop_requested = False
        self.available_modes[default].start()
        self.request_mode = rospy.Service('request_mode', ModeSwitcher, self.runService)


    def runService(self,req):

        resp = ModeSwitcherResponse()

        print (req.request_mode.data, type(req.request_mode.data))
        if req.request_mode.data == "Stop":
            print "here"
            self.is_stop_requested = True
            resp.succeeded.data = True
            return resp

        for key, value in self.modes.iteritems():
            if key == req.request_mode.data:
                self.available_modes[key] = RosLaunchMode(package = value['package'],
                                                      route_to_launch_file = value['path_to_file'])

        if req.request_mode.data == "SaveMap":
            self.available_modes[req.request_mode.data].start()
            rospy.loginfo("Saving map")
            rospy.sleep(3)
            return resp

        self.available_modes[self.current_mode].stop()
        self.available_modes[req.request_mode.data].start()
        self.current_mode = req.request_mode.data
        print("DONE")

        return resp
