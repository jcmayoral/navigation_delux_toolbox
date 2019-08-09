import numpy as np
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray

# define state Monitor
class Monitor(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['END_MONITOR'],
                              input_keys=['stop_bag','acc_cum', 'cam_cum', 'odom_cum', 'imu_cum', 'lidar_cum', 'mic_cum', 'overall_cum', 'result_cum'],
                              output_keys=['stop_bag','acc_cum', 'cam_cum', 'odom_cum', 'imu_cum', 'lidar_cum', 'mic_cum', 'overall_cum', 'result_cum'])
        rospy.Subscriber("/finish_reading", String, self.fb_cb)
        rospy.Subscriber("/ground_truth", PoseArray, self.groundtruth_cb)
        rospy.Subscriber("/objects_detected", PoseArray, self.detection_cb, queue_size = 1000)

        self.detected_objects = list()
        self.gt_objects = list()

        self.current_counter = 0
        self.overall_count = 0

        self.ground_truth_count = 0
        self.ground_truth = list()
        self.detection_times = list()
        self.false_positives_count = 0
        self.false_negative_count = 0
        self.truth_positives = 0

        self.delays = list()
        self.overall_cum = list()

        self.start_time = rospy.rostime.get_rostime().to_sec()

    def detection_cb(self,msg):
        curr_time = rospy.rostime.get_rostime().to_sec()
        self.overall_count = self.overall_count + 1
        rospy.logerr("Somethin is detected Detection")
        rospy.logerr('curr_time %s',curr_time - self.start_time)
        self.detection_times.append(curr_time - self.start_time)

        for p in msg.poses:
            self.detected_objects.append(p)

    def fb_cb(self,msg):
        rospy.logerr("Monitor has been notified that rosbag has ended")
        self.stop_bag_request = True

    def groundtruth_cb(self,msg):
        curr_time = rospy.rostime.get_rostime().to_sec()
        self.overall_count = self.overall_count + 1
        rospy.logerr("Somethin is detected Detection")
        rospy.logerr('curr_time %s',curr_time - self.start_time)
        self.detection_times.append(curr_time - self.start_time)

        curr_time = rospy.rostime.get_rostime().to_sec()
        rospy.logerr("GROUND TRUTH: %s", msg.stamp.secs)
        rospy.loginfo('curr_time %s',curr_time - self.start_time)
        self.ground_truth_count = self.ground_truth_count + 1
        self.ground_truth.append(curr_time - self.start_time)

        for p in msg.poses:
            self.gt_objects.append(p)

    def execute(self, userdata):

        self.stop_bag_request = False
        rospy.loginfo('Executing state MONITORING')
        #rospy.sleep(20)#TODO
        self.start_time = rospy.rostime.get_rostime().to_sec()

        while not self.stop_bag_request:
            pass #TODO
        rospy.logerr("WORK")

        rospy.sleep(0.2)
        self.first_collision = False

        #print ("NEXT_MONITOR")
        print ("Ground Truths " , self.ground_truth)
        print ("Truth Positives " , self.truth_positives)

        collisions_detected = len(self.detection_times)
        print ('Samples with Obejcts [%s]' % ', '.join(map(str, self.detection_times)))
        print ("Collisions Detected " , collisions_detected)

        if collisions_detected > 0: # If collisions were detecteds

            for gt in self.ground_truth:
                delay = np.abs(np.array(self.detection_times)-gt).min() #closest collisions -> Ground Truth
                arg_delay = np.abs(np.array(self.detection_times)-gt).argmin() # index of the closes collision detecteds

                print('Closest to ', gt ," is ", delay) # Closest delay print

                if delay < 0.5: #if delay is less than 1 second then it is considered as a true positive
                    self.truth_positives = self.truth_positives + 1
                    self.delays.append(delay)
                    collisions_detected = collisions_detected - 1 #our counter decreased
                    self.detection_times.remove(self.detection_times[arg_delay]) # removing from the detected collsiions
                    #self.false_positives_count = self.false_positives_count + collisions_detected - 1 # all detections minus the closest are false positives
                    for  c in self.detection_times:
                        if 0.8> (c - gt) > 0.5: # if a collision detected is less than 0.7 seconds it is considered as a false positiv
                            print ("FOUND COllision Close to Ground Truth " , c - gt)
                            self.detection_times.remove(c) # removing from the detected collsiions
                            collisions_detected = collisions_detected - 1 #our counter decreased
                else:
                    print ("Collision NOT FOUND")
                    self.false_negative_count = self.false_negative_count + 1

        else: #The ground truth was not detected
            print ("ANY COLLISION DETECTED, false negatives added ", len(self.ground_truth))
            self.false_negative_count = self.false_negative_count + len(self.ground_truth)


        print ("FALSE POSITIVES FOUND : ", collisions_detected )
        self.false_positives_count = collisions_detected + self.false_positives_count #if best delay is bigger than 1 second then the collision was not detected

        self.ground_truth = list()
        self.detection_times = list()
        return 'END_MONITOR'
