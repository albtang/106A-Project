#!/usr/bin/env python
import os
import sys
import rospy
import roslib
roslib.load_manifest("moveit_python")
import numpy as np
import cv2
import cmath
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from color_segmentation import *

OBJECTS_PUB_TOPIC = 'detected_objects'
IMAGE_SUB_TOPIC = '/cameras/right_hand_camera/image'
AR_TRACKER_TOPIC = '/ar_pose_marker'

width = 36
length = 30
conversion = 2.54/100
HOMOGRAPHY_DEST = np.array([[0.0,0.0],[3600.0, 0.0], [0.0, 3000.0],[3600.0, 3000.0]])

COLORS = ["red", "purple", "blue"]
bridge = CvBridge()

class RealWorldPos:
    def __init__(self, x, y, z, color):
        self.x = x
        self.y = y
        self.z = z
        self.color = color

class BaxterImage():
    def __init__(self):
        self.corners = None
        self.H = None
        self.numConverge = 0
        self.centers = [] * len(COLORS)
        self.ar_pos = None
        self.times_centers_found = 0
        self.position_publisher = rospy.Publisher(OBJECTS_PUB_TOPIC, PoseArray, queue_size=10)


    def image_callback(self, img_msg):
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
            # if not self.corners:
            thresh = grayscale(cv_image)
            cnt = contours(thresh, cv_image)
            crnrs = corners(cv_image, cnt)
            self.H, status = cv2.findHomography(np.array(crnrs), np.array(HOMOGRAPHY_DEST))
            # print(type(self.H))
            # print(self.H)
            self.corners = crnrs
            if self.numConverge < 10:
                homography, status = cv2.findHomography(np.array(self.corners), np.array(HOMOGRAPHY_DEST))
                if np.allclose(self.H, homography, atol=.1):
                    self.numConverge += 1
                else:
                    self.numConverge = 0
                self.H = homography
        except CvBridgeError as e:
            print(e)

        # if self.times_centers_found < 10:
        # mask = segment_by_color(cv_image, "purple")
        # cnts = contours(mask, cv_image)
        # self.centers.append(np.array(midpoint(cv_image, cnts)))
        # self.homography(cv_image)
        # self.times_centers_found += 1

        # purple_center = np.dot(self.H, np.array([self.centers[0][0], self.centers[0][1], 1]))
        # print((purple_center * conversion) / 100)
        # purple_center = (purple_center * conversion) / 100
        if self.numConverge >= 10:
            positions = PoseArray()
            positions.poses = []
            positions.header.frame_id = ' '.join(COLORS)
            for i, color in enumerate(COLORS): 
                mask = segment_by_color(cv_image, color)
                cnts = contours(mask, cv_image)
                mp = midpoint(cv_image, cnts)
                cntr = np.dot(self.H, np.array([mp[0], mp[1], 1]))
                cntr_xy = (cntr * conversion) / 100
                pose = Pose()
                pose.position.x = cntr_xy[0]
                pose.position.y = cntr_xy[1]
                pose.position.z = 1

                positions.poses.append(pose)
                # self.centers[i] = np.array([cntr_xy[0], cntr_xy[1], 1])
                # self.centers[i] = np.array([cntr_xy[0] + self.ar_pos.x, cntr_xy[1] + self.ar_pos.y, self.ar_pos.z])
                self.homography(cv_image)
            # print(self.centers)
            # return positions
            self.position_publisher.publish(positions)
            
        cv2.imshow('image', cv_image)
        cv2.waitKey(1)

    def ar_callback(self, ar_msg):
        if self.ar_pos is None:
            self.ar_pos = np.array(ar_msg.markers[0].pose.pose.position)
        # print(ar_msg.markers)
        # print(self.ar_pos)

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
        image_subscriber = rospy.Subscriber(IMAGE_SUB_TOPIC, Image, self.image_callback)
        ar_subscriber = rospy.Subscriber(AR_TRACKER_TOPIC, AlvarMarkers, self.ar_callback)
        rospy.spin()
        cv2.destroyAllWindows()

baxterImage = BaxterImage()
baxterImage.main()