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

OBJECTS_PUB_TOPIC = 'detected_objects'
IMAGE_SUB_TOPIC = '/cameras/right_hand_camera/image'

#The following are the images of each singular piece in order to perform sifting and feature extraction
red_piece = cv2.imread('img path here')
orange_piece = cv2.imread('img path here')
yellow_piece = cv2.imread('img path here')
green_piece = cv2.imread('img path here')
black_piece = cv2.imread('img path here')
purple_piece = cv2.imread('img path here')
blue_piece = cv2.imread('img path here')
MIN_MATCH_COUNT = 10

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

def imageCallback(img_msg):
    return None

def sift_transform_object(img_src, desired_obj):
    # https://docs.opencv.org/3.4/d1/de0/tutorial_py_feature_homography.html
    # img_src is the image we receive from our camera
    # desired_obj is the piece we are trying to extract from the img_src

    #Initiate SIFT detector
    sift = cv2.xfeatures2d.SIFT_create()

    kp1, des1 = sift.detectAndCompute(desired_obj, None) # queryImage
    kp2, des2 = sift.detecrAndCompute(img_src, None) # trainImage

    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1, des2, k=2)

    # Store all good matches
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1,1,2)

        M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()

        h, w, d = img1.shape
        pts =  np.float32([[0,0], [0,h-1], [w-1,h-1], [w-1,0]]).reshape(-1,1,2)
        dst = cv.perspectiveTransform(pts, M)

        img2 = cv.polylines(img2, [np.int32(dst)], True, 255, 3, cv.LINE_AA)

    else:
        print("Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))
        matchesMask = None

    draw_params = dict(matchColor = (0, 255, 0),
                        singlePointColor = None, 
                        matchesMask = matchesMask,
                        flags = 2)

    img3 = cv.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)
    plt.imshow(img3)
    plt.show()


def getPosition():
    return None

def get3DPosition():
    return None

def getRealPosition():
    return None

def filterColor():
    return None

def main(args):
    rospy.init_node('object_positions', anonymous=True)
    position_publisher = rospy.Publisher(OBJECTS_PUB_TOPIC, PoseArray, queue_size=10)
    image_subscriber = rospy.Subscriber(IMAGE_SUB_TOPIC, 1, imageCallback)
    return None

if __name__ == 'main':
    main(sys.argv)