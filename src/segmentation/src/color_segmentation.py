#!/usr/bin/env python
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

COLORS = {
    "red_low": [[0, 100, 100], [10, 255, 255]],
    "red_high": [[160, 100, 100], [179, 255, 255]],
    # "orange": [[15, 100, 100], [20, 255, 255]],
    # "yellow": [[25, 100, 100], [45, 255, 255]],
    "blue": [[97,79,50], [123,149,90]],
    "green": [[59,44,94], [93,92,150]],
    # "black": [[0, 0, 0], [180, 230, 30]],
    "purple": [[130, 70, 70], [160, 255, 255]],
    # "wood": [[0, 0, 100], [21, 150, 255]],
    # "bleach": [[0, 0, 100], [0, 50, 255]],
    # "white": [[0,30,205], [166,100,255]]
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

# def mask_white(mask, image):
#     masked_img = cv2.bitwise_and(image, image, mask = mask)
#     masked_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
#     masked_img = cv2.inRange(image, np.array(COLORS['white'][0]), np.array(COLORS['white'][1]))
#     return masked_img

def grayscale(image):
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(img,200,255,cv2.THRESH_BINARY_INV)
    return thresh
    # plt.imshow(thresh)
    # plt.show()

def contours(mask, cv_image):
    image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours[0]
    max_area = cv2.contourArea(cnt)
    for cont in contours:
        if cv2.contourArea(cont) > max_area and cv2.contourArea(cont) <= cv_image.shape[0]*cv_image.shape[1]*0.99:
            cnt= cont
            max_area = cv2.contourArea(cont)
    perimeter = cv2.arcLength(cnt,True)
    epsilon = 0.01*cv2.arcLength(cnt,True)
    approx = cv2.approxPolyDP(cnt,epsilon,True)

    return approx

def corners(img, cnts):
    left = tuple(cnts[cnts[:, :, 0].argmin()][0])
    right = tuple(cnts[cnts[:, :, 0].argmax()][0])
    top = tuple(cnts[cnts[:, :, 1].argmin()][0])
    bottom = tuple(cnts[cnts[:, :, 1].argmax()][0])
    cv2.drawContours(img, [cnts], -1, (0, 255, 255), 2)
    cv2.circle(img, left, 8, (0, 0, 255), -1) #red
    cv2.circle(img, right, 8, (0, 255, 0), -1) #green
    cv2.circle(img, top, 8, (255, 0, 0), -1) #blue
    cv2.circle(img, bottom, 8, (255, 255, 0), -1) #cyan
    # print([left, right, top, bottom])

    return [bottom, left, top, right]
 
    # show the output image
    # plt.imshow(img)
    # plt.show()

def midpoint(img, cnts):
    min_x = cnts[cnts[:, :, 0].argmin()][0][0]
    max_x = cnts[cnts[:, :, 0].argmax()][0][0]
    min_y = cnts[cnts[:, :, 1].argmin()][0][1]
    max_y = cnts[cnts[:, :, 1].argmax()][0][1]
    x_coord = min_x + (max_x - min_x) / 2
    y_coord = min_y + (max_y - min_y) / 2

    cv2.circle(img, tuple([x_coord, y_coord]), 8, (255, 255, 255), -1)
    return (x_coord, y_coord)
    # plt.imshow(img)
    # plt.show()


if __name__ == '__main__':
    test = './testdata/test.jpg'
    mult = './testdata/multiple_pieces.jpg'
    org_yel = './testdata/org_yel.jpg'
    crossed = './testdata/crossed.jpg'
    cross = './testdata/cross.jpg'
    wood = './testdata/wood.jpg'
    table = './testdata/table2.jpg'
    baxter = './testdata/baxter.png'
    baxter2 = './testdata/baxter2.png'
    less_pieces = './testdata/less_pieces.png'
    pieces = './testdata/pieces.png'
    image = cv2.imread(pieces)
    # grayscale(image)
    # thresh = grayscale(image)
    # contours = contours(thresh, image)
    mask = segment_by_color(image, "red")
    # mask = segment_by_color(image, "wood")
    cnts = contours(mask, image)
    crnrs = corners(image, cnts)
    midpoint(image, cnts)
   