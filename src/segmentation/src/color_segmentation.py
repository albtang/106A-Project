#!/usr/bin/env python
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

COLORS = {
    "red_low": [[0, 100, 100], [10, 255, 255]],
    "red_high": [[160, 100, 100], [179, 255, 255]],
    "orange": [[15, 100, 100], [20, 255, 255]],
    "yellow": [[25, 100, 100], [45, 255, 255]],
    "blue": [[110,30,30], [130,255,255]],
    "green": [[40,60,60], [70,255,255]],
    "black": [[0, 0, 0], [180, 230, 30]],
    "purple": [[140, 100, 100], [160, 255, 255]],
    "wood": [[10, 50, 175], [21, 150, 240]]
}

def segment_by_color(image, color):
    # img = cv2.imread(image)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = None
    if color == "red":
        low = color + "_low"
        high = color + "_high"
        mask1 = cv2.inRange(img, np.array(COLORS[low][0]), np.array(COLORS[low][1]))
        mask2 = cv2.inRange(img, np.array(COLORS[high][0]), np.array(COLORS[high][1]))
        mask = mask1 + mask2
    else:
        mask = cv2.inRange(img, np.array(COLORS[color][0]), np.array(COLORS[color][1]))
    # return mask
    masked_img = cv2.bitwise_and(cv_image, cv_image, mask = mask)
    image, contours, hierarchy = cv2.findContours(masked_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    img = cv2.drawContours(img, contours, -1, (0,255,0), 3)
    # plt.imshow(mask, cmap='gray')
    # plt.title("Segmentation by %s" % color)
    # plt.show()

if __name__ == '__main__':
    test = './testdata/test.jpg'
    mult = './testdata/multiple_pieces.jpg'
    org_yel = './testdata/org_yel.jpg'
    crossed = './testdata/crossed.jpg'
    cross = './testdata/cross.jpg'
    wood = './testdata/wood.jpg'
    image = cv2.imread(wood)
    segment_by_color(image, "wood")
   