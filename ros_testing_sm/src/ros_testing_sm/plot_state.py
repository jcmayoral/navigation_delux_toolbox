import rospy
import smach
import smach_ros

class Plotter(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['PLOT_DONE'],
                             input_keys=['data_in', 'x_array'])
    def execute(self, userdata):
        rospy.loginfo('Executing Plotter')
        return 'PLOT_DONE'
