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
    "wood": [[0, 0, 100], [21, 150, 255]],
    "bleach": [[0, 0, 100], [0, 50, 255]],
    "white": [[0,30,205], [166,100,255]]
}

def segment_by_color(image, color):
    # img = cv2.imread(image)
    img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = None
    if color == "red":
        low = color + "_low"
        high = color + "_high"
        mask1 = cv2.inRange(img, np.array(COLORS[low][0]), np.array(COLORS[low][1]))
        mask2 = cv2.inRange(img, np.array(COLORS[high][0]), np.array(COLORS[high][1]))
        mask = mask1 + mask2
    else:
        mask = cv2.inRange(img, np.array(COLORS[color][0]), np.array(COLORS[color][1]))
    return mask
    plt.imshow(mask, cmap='gray')
    plt.title("Segmentation by %s" % color)
    plt.show()

def mask_white(mask, image):
    masked_img = cv2.bitwise_and(image, image, mask = mask)
    masked_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
    masked_img = cv2.inRange(image, np.array(COLORS['white'][0]), np.array(COLORS['white'][1]))
    return masked_img

def contours(mask, cv_image):
    image, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # cnt_image = cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
    cnt_image = np.zeros(mask.shape, np.uint8)
    for cnt in contours:
        epsilon = 0.1*cv2.arcLength(cnt,True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        cv2.drawContours(cnt_image, approx, -1, (0,255,0), 3)
    # cnts = contours[0]
    # print(len(contours))
    # x,y,w,h = cv2.boundingRect(cnts)
    # cv2.circle(image,(x,y), 3, (0,0,255), -1)
    # cv2.circle(image,(x+w,y), 3, (0,0,255), -1)
    # cv2.circle(image,(x+w,y+h), 3, (0,0,255), -1)
    # cv2.circle(image,(x,y+h), 3, (0,0,255), -1)
    # # # cv2.imshow("Corners of grid", image)
    plt.imshow(cnt_image)
    plt.show()
    # return contours, cnt_image

def corners(img, color):
    mask = segment_by_color(img, color)
    gray = np.float32(mask)
    dst = cv2.cornerHarris(gray, 2, 3, 0.04)
    # cv2.imshow('dst',img)
    # if cv2.waitKey(0) & 0xff == 27:
    #     cv2.destroyAllWindows()

if __name__ == '__main__':
    test = './testdata/test.jpg'
    mult = './testdata/multiple_pieces.jpg'
    org_yel = './testdata/org_yel.jpg'
    crossed = './testdata/crossed.jpg'
    cross = './testdata/cross.jpg'
    wood = './testdata/wood.jpg'
    table = './testdata/table2.jpg'
    baxter = './testdata/baxter.png'
    baxter2 = './testdata/baxter2.jpg'
    image = cv2.imread(baxter2)
    # segment_by_color(image, "wood")
    mask = segment_by_color(image, "wood")
    contours(mask, image) 
    # corners(image, "wood")
   