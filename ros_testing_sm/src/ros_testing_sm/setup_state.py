import rospy
import smach
import smach_ros
from roslaunch_mode_switcher.srv import *

class Setup(smach.State):
    def __init__(self,max_window_size=75,step=5):
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'PRINT_RESULTS', 'FINISH_SM'],
                             input_keys=['stop','results_shown', 'counter_in', 'lidar_results', 'result_cum', 'results_', 'x_array'],
                             output_keys=['stop','results_shown','counter_out','current_mode','acc_results', 'cam_results', 'odom_results', 'imu_results', 'lidar_results', 'mic_results', 'result_cum', 'results_', 'x_array'])
        self.selected_modes = ["LidarCPU", "LidarGPU"]#["Camera", "LidarCPU", "LidarGPU"]
        self.step = step
        self.is_first_time = True
        rospy.loginfo("waiting rqeust_mode service")
        rospy.wait_for_service('request_mode')
        self.mode_switch_client = rospy.ServiceProxy('request_mode', ModeSwitcher)
        rospy.loginfo("request_mode service started")

        rospy.sleep(0.2)

    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')
        rospy.sleep(0.5)

        userdata.stop = False

        if len(self.selected_modes) == 0 and userdata.results_shown:
            userdata.stop = True
            swap_mode = ModeSwitcherRequest()
            swap_mode.request_mode.data = "Stop"
            print(self.mode_switch_client(swap_mode))
            return 'FINISH_SM'

        if not userdata.results_shown:
            #userdata.results_['accel'] = userdata.acc_results
            #userdata.results_['cam1'] = userdata.cam_results
            #userdata.results_['odom'] = userdata.odom_results
            #userdata.results_['imu'] = userdata.imu_results
            userdata.results_['lidar'] = userdata.lidar_results
            #userdata.results_['mic'] = userdata.mic_results
            return 'PRINT_RESULTS'

        swap_mode = ModeSwitcherRequest()
        swap_mode.request_mode.data = self.selected_modes.pop(0)
        print(self.mode_switch_client(swap_mode))
        userdata.results_shown = False

        userdata.current_mode = swap_mode.request_mode.data
        userdata.x_array.append(userdata.counter_in)
        userdata.counter_out = userdata.counter_in + self.step
        self.is_first_time = False
        return 'SETUP_DONE'
