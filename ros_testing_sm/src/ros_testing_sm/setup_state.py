import rospy
import smach
import smach_ros

class Setup(smach.State):
    def __init__(self,max_window_size=75,step=5):
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'PRINT_RESULTS', 'FINISH_SM'],
                             input_keys=['counter_in','acc_results', 'cam_results', 'odom_results', 'imu_results', 'lidar_results', 'mic_results', 'result_cum', 'results_', 'x_array'],
                             output_keys=['counter_out','acc_results', 'cam_results', 'odom_results', 'imu_results', 'lidar_results', 'mic_results', 'result_cum', 'results_', 'x_array'])
        #HERE I should merge the roslaunch_modes_node
        self.step = step
        self.is_first_time = True
        self.end = False

        rospy.sleep(0.2)

    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')
        rospy.sleep(0.5)
        if self.end:
            return 'FINISH_SM'

        if self.is_first_time:
            userdata.x_array.append(userdata.counter_in)
            userdata.counter_out = userdata.counter_in + self.step
            self.is_first_time = False
            return 'SETUP_DONE'
        else:
            self.end = True
            userdata.results_['accel'] = userdata.acc_results
            userdata.results_['cam1'] = userdata.cam_results
            userdata.results_['odom'] = userdata.odom_results
            userdata.results_['imu'] = userdata.imu_results
            userdata.results_['lidar'] = userdata.lidar_results
            userdata.results_['mic'] = userdata.mic_results
            return 'PRINT_RESULTS'
