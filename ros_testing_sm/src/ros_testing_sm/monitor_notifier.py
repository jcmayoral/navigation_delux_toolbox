import rospy
import smach
import smach_ros
from std_msgs.msg import Empty

class MonitorNotifier(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['STOP_READING', 'NOTIFICATION_SEND'],
                             input_keys=['stop'],
                             output_keys=['stop'])
        #rospy.spin()
        self.monitor_reset_pub = rospy.Publisher('/sm_reset', Empty, queue_size=1)
        rospy.sleep(0.2)

    def execute(self, userdata):
        rospy.loginfo('Notifyinf  Monitor that bag has finished')
        self.monitor_reset_pub.publish(Empty())

        #HACK ...should have fb ...Service?
        rospy.sleep(2)

        if not userdata.stop:
            userdata.stop = True
            return 'NOTIFICATION_SEND'

        return 'STOP_READING'
