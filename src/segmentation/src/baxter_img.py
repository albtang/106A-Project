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

HOMOGRAPHY_DEST = [[0, 0], [0, 0.9082], [0.605, 0], [0.605, 0.9082]]

bridge = CvBridge()

def image_callback(img_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    mask = segment_by_color(cv_image, "wood")
    mask = mask_white(mask, cv_image)
    conts, cont_img = contours(mask, cv_image)
    cv2.imshow('image', cont_img)
    cv2.waitKey(1)

def homography(img1, img2):
    return None

def getRealPosition():
    return None

def main():
    rospy.init_node('object_positions', anonymous=True)
    position_publisher = rospy.Publisher(OBJECTS_PUB_TOPIC, PoseArray, queue_size=10)
    image_subscriber = rospy.Subscriber(IMAGE_SUB_TOPIC, Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()

main()