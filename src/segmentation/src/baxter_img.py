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
HOMOGRAPHY_DEST = np.array([[0.0,0.0],[3600.0, 0.0], [0.0, 3000.0],[3600.0, 3000.0]])

# right_hand_camera values
# - Translation: [0.403, 0.288, 0.225]
# - Rotation: in Quaternion [0.424, 0.867, -0.133, 0.228]
#             in RPY (radian) [-3.098, 0.532, 2.244]
#             in RPY (degree) [-177.491, 30.484, 128.585]

bridge = CvBridge()

class BaxterImage():
    def __init__(self):
        self.corners = None
        self.H = None
        self.numConverge = 0

    def image_callback(self, img_msg):
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
            # if not self.corners:
            thresh = grayscale(cv_image)
            cnt = contours(thresh, cv_image)
            crnrs = corners(cv_image, cnt)
            self.H, status = cv2.findHomography(np.array(crnrs), np.array(HOMOGRAPHY_DEST))
            print(type(self.H))
            print(self.H)
            self.corners = crnrs
            if self.numConverge < 10:
                homography, status = cv2.findHomography(np.array(self.corners), np.array(HOMOGRAPHY_DEST))
                if np.allclose(self.H, homography, atol=.1):
                    self.numConverge += 1
                self.H = homography
        except CvBridgeError as e:
            print(e)
        mask = segment_by_color(cv_image, "purple")
        self.homography(cv_image)
        cv2.imshow('image', cv_image)
        
        cv2.waitKey(1)

    def homography(self, img):
        #---- 4 corner points of the bounding box
        pts_src = np.array(self.corners)

        #---- 4 corner points of the black image you want to impose it on
        pts_dst = HOMOGRAPHY_DEST #np.array([[0.0,3000.0],[3600.0, 3000.0],[ 0.0,0.0],[3600.0, 0.0]])

        #---- forming the black image of specific size
        im_dst = np.zeros((3001, 3601, 3), np.uint8)

        #---- transforming the image bound in the rectangle to straighten
        image = cv2.warpPerspective(img, self.H, (im_dst.shape[1],im_dst.shape[0]))
        cv2.imwrite("image.jpg", image)

        #find midpoint of pieces
        #use homography matrix to transform into x, y in straightened image

        #use ar tag to find x, y in real life

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