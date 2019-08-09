from __future__ import print_function
import rospy
import smach
import smach_ros
import rosbag
import importlib
from geometry_msgs.msg import AccelStamped, Twist, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, Imu, PointCloud2
from std_msgs.msg import Empty, String
from audio_common_msgs.msg import AudioData

# define state ReadBag
class MyBagReader(smach.State):
    def __init__(self):
        self.myPublishers = dict()
        self.finish_pub = rospy.Publisher("finish_reading", String, queue_size=1)

        smach.State.__init__(self,
                             outcomes=['END_READER'],
                             input_keys=['foo_counter_in', 'shared_string', 'path'],
                             output_keys=['foo_counter_out'])

    def load_topics_and_types(self):
        topics_and_types = self.bag.get_type_and_topic_info()
        list_zip = zip(topics_and_types.topics.keys(),topics_and_types.msg_types.keys())

        for topic_name, msg_type in list_zip:
            module_name, class_type = msg_type.split('/')
            m = importlib.import_module(module_name+".msg")
            publisher = rospy.Publisher(topic_name,  getattr(m, class_type), queue_size=10)
            self.myPublishers[topic_name] = publisher

        rospy.loginfo("FINISH")
        rospy.sleep(1)

    def execute(self, userdata):
        rospy.loginfo('Starting ROSBAG')
        try:
            file_name = userdata.path + userdata.shared_string + ".bag"
            self.bag = rosbag.Bag(file_name)
            print ("file_name" , file_name )
        except:
            rospy.logerr("Error Open File " + str(file_name))
            fb = String()
            fb.data = "END_BAG"
            self.finish_pub.publish(fb)
            rospy.sleep(2)
            return 'END_READER'

        self.load_topics_and_types()

        for topic, m, t in self.bag.read_messages():
            try:
                self.myPublishers[topic].publish(m)
                #rospy.sleep(0.1)
            except:
                pass

        fb = String()
        fb.data = "END_BAG"
        self.finish_pub.publish(fb)
        rospy.sleep(2)
        return 'END_READER'
