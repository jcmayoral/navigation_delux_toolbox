#! /usr/bin/python
from ros_testing_sm.plot_objects import ObjectVisualizer
import rospy

if __name__ == '__main__':
    rospy.init_node('object_visualizer_node')
    object_visualizer = ObjectVisualizer()
    rospy.spin()
