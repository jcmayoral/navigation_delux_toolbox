import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PoseArray

class ObjectVisualizer():
    def __init__(self):
        rospy.Subscriber("detected_objects", PoseArray, self.poses_cb)
        self.f = plt.figure()
        self.ax = plt.axes()
        self.ax.set_title('Results')
        self.ax.legend("True")
        plt.show()


    def poses_cb(self, objects):
        for p in objects.poses:
            self.ax.scatter(p.position.x, p.position.y)
            plt.draw()
