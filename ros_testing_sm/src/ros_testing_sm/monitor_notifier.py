import rospy
import smach
import smach_ros
from std_msgs.msg import Empty

class MonitorNotifier(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['STOP_READING', 'MONITOR_NOTIFIED'],
                             input_keys=['stop'],
                             output_keys=['stop'])
        #rospy.spin()
        self.monitor_start_pub = rospy.Publisher('/rosbag_play_event', Empty, queue_size=1)
        self.stop = False
        rospy.sleep(0.2)

    def execute(self, userdata):
        if not self.stop:
            self.monitor_start_pub.publish()
            self.stop = True
            return 'MONITOR_NOTIFIED'

        self.stop = False
        self.monitor_start_pub.publish()
        return 'STOP_READING'
