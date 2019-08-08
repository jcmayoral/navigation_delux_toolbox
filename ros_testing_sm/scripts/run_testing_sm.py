#! /usr/bin/python
from __future__ import print_function
import rospy
import smach
import smach_ros
import math
from geometry_msgs.msg import AccelStamped, Twist, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, Imu
from std_msgs.msg import Empty, String, Header
#from fusion_msgs.msg import sensorFusionMsg, controllerFusionMsg, monitorStatusMsg
from dynamic_reconfigure.client import Client

import numpy as np
import matplotlib.pyplot as plt
from ros_testing_sm.sm_tools import start_sm


class Setup(smach.State):
    def __init__(self,max_window_size=75,step=5):
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'FINISH'],
                             input_keys=['counter_in','acc_results', 'cam_results', 'odom_results', 'imu_results', 'lidar_results', 'mic_results', 'result_cum', 'results_', 'x_array'],
                             output_keys=['counter_out','acc_results', 'cam_results', 'odom_results', 'imu_results', 'lidar_results', 'mic_results', 'result_cum', 'results_', 'x_array'])
        #HERE I should merge the roslaunch_modes_node
        self.step = step
        self.is_first_time = True

        rospy.sleep(0.2)

    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')
        rospy.sleep(0.5)

        if self.is_first_time:
            userdata.x_array.append(userdata.counter_in)
            userdata.counter_out = userdata.counter_in + self.step
            self.is_first_time = False
            return 'SETUP_DONE'
        else:
            userdata.results_['accel'] = userdata.acc_results
            userdata.results_['cam1'] = userdata.cam_results
            userdata.results_['odom'] = userdata.odom_results
            userdata.results_['imu'] = userdata.imu_results
            userdata.results_['lidar'] = userdata.lidar_results
            userdata.results_['mic'] = userdata.mic_results
            return 'FINISH'

class Plotter(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['PLOT_DONE'],
                             input_keys=['data_in', 'x_array'])
    def execute(self, userdata):
        rospy.loginfo('Executing Plotter')
        return 'PLOT_DONE'

# define state Monitor
class Monitor(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['NEXT_MONITOR', 'END_MONITOR'],
                              input_keys=['acc_cum', 'cam_cum', 'odom_cum', 'imu_cum', 'lidar_cum', 'mic_cum', 'overall_cum', 'result_cum'],
                              output_keys=['acc_cum', 'cam_cum', 'odom_cum', 'imu_cum', 'lidar_cum', 'mic_cum', 'overall_cum', 'result_cum'])
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
        #print ("CB", msg)
        self.next_bag_request = True
        self.stop_bag_request = False

        if msg.data == "NEXT_BAG":

            print ("current_counter" , self.current_counter)
            #self.current_counter = 0
        else:#FINISH
            #print ("/n")
            print ("Ground Truth Count ", self.ground_truth_count)
            print ("Truth Positves Count ", self.truth_positives)

            print (self.delays)
            print ("Average Reaction Time ", np.nanmean(self.delays))
            print ("False Positives ", self.false_positives_count)
            print ("False Negatives ", self.false_negative_count)
            print ("Total collisions detected by all observers: " , self.current_counter)
            print ("SF Total Collisions Detected" , self.overall_count)

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

        self.next_bag_request = False
        rospy.loginfo('Executing state MONITORING')
        #rospy.sleep(20)#TODO
        self.start_time = rospy.rostime.get_rostime().to_sec()

        while not self.next_bag_request:
            pass #TODO

        rospy.sleep(0.2)
        self.next_bag_request = False
        self.first_collision = False

        if self.stop_bag_request:

            #userdata.acc_cum.append(self.observer_counter['accelerometer_1'])
            #userdata.cam_cum.append(self.observer_counter['cam1'])
            #userdata.odom_cum.append(self.observer_counter['odom'])
            #userdata.imu_cum.append(self.observer_counter['imu_1'])
            #userdata.lidar_cum.append(self.observer_counter['lidar_1'])
            #userdata.mic_cum.append(self.observer_counter['mic_1'])

            return 'END_MONITOR'
        else:
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
            return 'NEXT_MONITOR'



if __name__ == '__main__':
    rospy.init_node('ros_testing_sm')
    start_sm("/media/datasets/nibio_summer_2019/long_video/", "2019-05-21-17-28-51", Monitor, Setup, Plotter, time_limit = 15, max_bag_file = 110)
    #start_sm("/home/jose/data/stomach_collisions/", "stomach_collision_", Monitor, Setup, Plotter, time_limit = 15, max_bag_file = 60)
