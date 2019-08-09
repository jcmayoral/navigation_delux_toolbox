import rospy
import smach
import smach_ros
from roslaunch_mode_switcher.srv import *

class Setup(smach.State):
    def __init__(self,max_window_size=75,step=5):
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'PRINT_RESULTS', 'FINISH_SM'],
                             input_keys=['results_shown', 'lidar_results', 'results_', 'x_array'],
                             output_keys=['results_shown','current_mode','lidar_results', 'results_', 'x_array'])
        self.selected_modes = ["LidarCPU", "LidarGPU"]#["Camera", "LidarCPU", "LidarGPU"]
        self.step = step
        rospy.loginfo("waiting mode swtitcher service")
        rospy.wait_for_service('request_mode')
        self.mode_switch_client = rospy.ServiceProxy('request_mode', ModeSwitcher)
        rospy.loginfo("request_mode service started")

        rospy.sleep(0.2)

    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')
        rospy.sleep(0.5)

        if len(self.selected_modes) == 0 and userdata.results_shown:
            swap_mode = ModeSwitcherRequest()
            swap_mode.request_mode.data = "Stop"
            print(self.mode_switch_client(swap_mode))
            return 'FINISH_SM'

        if not userdata.results_shown:
            userdata.results_['lidar'] = userdata.lidar_results
            return 'PRINT_RESULTS'

        swap_mode = ModeSwitcherRequest()
        swap_mode.request_mode.data = self.selected_modes.pop(0)
        print(self.mode_switch_client(swap_mode))
        userdata.results_shown = False

        userdata.current_mode = swap_mode.request_mode.data
        userdata.x_array.append(1)

        return 'SETUP_DONE'
