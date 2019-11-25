import os
import rospy
import roslib
roslib.load_manifest("moveit_python")
import numpy as np
import cv2
import cmath
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImgPos:
    def __init__(self, x, y, angle, size):
        self.x = x
        self.y = y
        self.angle = angle
        self.size = size

class WorldPos:
    def __init__(self, x, y, z, flag, angle, size):
        self.x = x
        self.y = y
        self.z = z
        self.flag = flag
        self.angle = angle
        self.size = size

rospy.init_node('object_positions', anonymous=True)
position_publisher = rospy.Publisher('detected_objects', PoseArray, queue_size=10)
image_subscriber = rospy.Subscriber('/cameras/right_hand_camera/image', 1, imageCallback)

def imageCallback():
    return None

def getPosition():
    return None

def get3DPosition():
    return None

def getRealPosition():
    return None

def filterColor():
    return None