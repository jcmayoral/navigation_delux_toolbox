#! /usr/bin/python
import rospy
import smach
import smach_ros
import rosbag
import math
from geometry_msgs.msg import AccelStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Empty, String
from dynamic_reconfigure.client import Client
#states
from bag_states import MyBagReader
from setup_state import Setup
from monitor_state import Monitor
from plot_state import Plotter
from monitor_notifier import MonitorNotifier

import matplotlib.pyplot as plt

def monitor_cb(ud, msg):
    rospy.logdebug("monitcb")
    return None

def init_dict():
    dic = dict()
    dic["min"] = list()
    dic["max"] = list()
    dic["mean"] = list()
    dic["std"] = list()
    return dic

# Create a SMACH state machine
def start_sm(path, common_string, max_window_size = 75, start_window = 2, step=5):
  sm = smach.StateMachine(outcomes=['END_SM'])

  #TO DELETE
  sm.userdata.window_size = start_window
  sm.userdata.window_size_array = list()
  sm.userdata.current_mode = "default"
  sm.userdata.results_shown = True
  sm.userdata.stop = False

  sm.userdata.stop_bag = False

  sm.userdata.results_ = dict()
  sm.userdata.acc_results = init_dict()
  #sm.userdata.cam_results = init_dict()
  #sm.userdata.odom_results = init_dict()
  #sm.userdata.imu_results = init_dict()
  sm.userdata.lidar_results = init_dict()
  #sm.userdata.mic_results = init_dict()

  reading_sm = smach.StateMachine(outcomes=['END_READING_SM'])
  reading_sm.userdata.path = path
  reading_sm.userdata.stop = sm.userdata.stop
  reading_sm.userdata.bag_family = common_string #TODO
  reading_sm.userdata.stop_bag  = sm.userdata.stop_bag


  monitoring_sm = smach.StateMachine(outcomes=['END_MONITORING_SM'])
  monitoring_sm.userdata.results_ = sm.userdata.results_
  monitoring_sm.userdata.lidar_results = sm.userdata.lidar_results

  monitoring_sm.userdata.stop = sm.userdata.stop
  monitoring_sm.userdata.stop_bag  = sm.userdata.stop_bag

  with reading_sm:
      smach.StateMachine.add('NOTIFY_MONITOR', MonitorNotifier(),
                     transitions={'MONITOR_NOTIFIED':'READING', 'STOP_READING':'END_READING_SM'})
      smach.StateMachine.add('READING', MyBagReader(),
                             transitions={'END_READER':'NOTIFY_MONITOR'},
                             remapping={'shared_string':'bag_family'})


  #montoring_sm.userdata.window_size_array = sm.window_size_array

  with monitoring_sm:
      smach.StateMachine.add('WAIT_TO_START', smach_ros.MonitorState("/rosbag_play_event", Empty, monitor_cb, input_keys=['stop'], output_keys=['stop']),
                              transitions={'invalid':'MONITOR', 'valid':'WAIT_TO_START', 'preempted':'WAIT_TO_START'})

      while rospy.Subscriber("/rosbag_play_event", Empty).get_num_connections() < 1:
          pass

      smach.StateMachine.add('MONITOR', Monitor(),
                     transitions={'END_MONITOR':'END_MONITORING_SM'},
                     remapping={'result_cum':'results_',
                                'acc_cum':'acc_results',
                                'cam_cum':'cam_results',
                                'odom_cum': 'odom_results',
                                'lidar_cum': 'lidar_results',
                                'imu_cum': 'imu_results',
                                'mic_cum': 'mic_results'})

  # Open the container
  with sm:
      smach.StateMachine.add('SETUP', Setup(max_window_size,step),
                     transitions={'SETUP_DONE':'CON', 'PRINT_RESULTS': 'PLOT_RESULTS', 'FINISH_SM': 'END_SM'},
                     remapping={'counter_in':'window_size',
                                'counter_out':'window_size',
                                'result_cum':'results_',
                                'acc_cum':'acc_results',
                                'cam_cum':'cam_results',
                                'odom_cum':'odom_results',
                                'imu_cum': 'imu_results',
                                'lidar_cum': 'lidar_results',
                                'mic_cum': 'mic_results',
                                'x_array': 'window_size_array'})

      #Concurrent
      sm_con = smach.Concurrence(outcomes=['END_CON'],
                                 default_outcome='END_CON',
                                 outcome_map={#'RESTART':
                                     #{ 'MONITORING_SM':'RESTART_MONITOR',
                                       #'READ_SM' : 'RESTART_READER'},
                                       'END_CON':
                                       {'READ_SM': 'END_READING_SM',
                                        'MONITORING_SM': 'END_MONITORING_SM'}})
      # Open the container
      with sm_con:
          # Add states to the container
          smach.Concurrence.add('READ_SM', reading_sm)
          smach.Concurrence.add('MONITORING_SM', monitoring_sm)

      #sm.userdata.data_in = monitoring_sm.userdata.acc_results #TODO
      sm.userdata.data_in = monitoring_sm.userdata.results_ #TODO

      smach.StateMachine.add('CON', sm_con,
                     transitions={'END_CON':'SETUP'})
      smach.StateMachine.add('PLOT_RESULTS', Plotter(),
                     transitions={'PLOT_DONE':'SETUP'},
                     remapping={'data_in': 'data_in',
                                'x_array': 'window_size_array'})

  # Execute SMACH plan
  #rospy.sleep(10)
  #outcome = sm.execute()
  #rospy.spin()

  #Instrospection
  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()
  # Execute the state machine
  outcome = sm.execute()
  #  for ctrl-c to stop the application
  plt.show()
  sis.stop()