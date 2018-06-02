#!/usr/bin/env python

from multi_map_navigation.elevator_manager import multimap_transition

if __name__ == '__main__':
    rospy.init_node("elevator_blast")

    blast = MultiMapControl()
    rospy.spin()
