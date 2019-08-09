#! /usr/bin/python
import rospy
from ros_testing_sm.sm_builder import start_sm

if __name__ == '__main__':
    rospy.init_node('ros_testing_sm')
    start_sm("/media/datasets/nibio_summer_2019/long_video/", "2019-05-21-17-28-51")
