#!/usr/bin/env python
import os
import sys
import rospy
import roslib
roslib.load_manifest("moveit_python")
import numpy as np
import cv2
import cmath
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from color_segmentation import *

OBJECTS_PUB_TOPIC = 'detected_objects'
IMAGE_SUB_TOPIC = '/cameras/right_hand_camera/image'

width = 36
length = 30
conversion = 2.54/100
HOMOGRAPHY_DEST = [[0, 0], [0, (length*2.54)/100], [(width*2.54)/100, 0], [(width*2.54)/100, (length*2.54)/100]]

# right_hand_camera values
# - Translation: [0.403, 0.288, 0.225]
# - Rotation: in Quaternion [0.424, 0.867, -0.133, 0.228]
#             in RPY (radian) [-3.098, 0.532, 2.244]
#             in RPY (degree) [-177.491, 30.484, 128.585]

bridge = CvBridge()
# homography = None
# numCorners = 0

class BaxterImage():
    def __init__(self):
        self.numCorners = 0
        self.homography = None

    def image_callback(self, img_msg):
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
            # if self.numCorners == 0:
            #     thresh = grayscale(cv_image)
            #     cnt = contours(thresh, cv_image)
            #     crnrs = corners(cv_image, cnt)
            #     self.homography = cv2.findHomography(np.array(crnrs), np.array(HOMOGRAPHY_DEST))
            #     # print(self.homography)
            #     self.numCorners += 1
        except CvBridgeError as e:
            print(e)
        # mask = segment_by_color(cv_image, "purple")
        cv2.imshow('image', cv_image)
        
        cv2.waitKey(1)

    def homography(self, img1, img2):
        return None

    def getRealPosition(self):
        return None

    def main(self):
        rospy.init_node('object_positions', anonymous=True)
        position_publisher = rospy.Publisher(OBJECTS_PUB_TOPIC, PoseArray, queue_size=10)
        image_subscriber = rospy.Subscriber(IMAGE_SUB_TOPIC, Image, self.image_callback)
        rospy.spin()
        cv2.destroyAllWindows()

baxterImage = BaxterImage()
baxterImage.main()