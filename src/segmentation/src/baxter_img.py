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
from color_segmentation import COLORS, segment_by_color

OBJECTS_PUB_TOPIC = 'detected_objects'
IMAGE_SUB_TOPIC = '/cameras/right_hand_camera/image'

HOMOGRAPHY_DEST = [[0, 0], [0, 0.9082], [0.605, 0], [0.605, 0.9082]]

def imageCallback(img_msg):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    mask = segment_by_color(cv_image, "wood")
    masked_img = cv2.bitwise_and(cv_image, cv_image, mask = mask)
    image, contours, hierarchy = cv2.findContours(masked_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return None

def homography(img1, img2):
    return None

def getRealPosition():
    return None

def main(args):
    # rospy.init_node('object_positions', anonymous=True)
    # position_publisher = rospy.Publisher(OBJECTS_PUB_TOPIC, PoseArray, queue_size=10)
    # image_subscriber = rospy.Subscriber(IMAGE_SUB_TOPIC, 1, imageCallback)
    
    
    return None

if __name__ == 'main':
    # main(sys.argv)