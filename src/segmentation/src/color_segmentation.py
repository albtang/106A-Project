#!/usr/bin/env python
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

# from scipy import ndimage
# from scipy.misc import imresize
# from skimage import filters
# from sklearn.cluster import KMeans

# this_file = os.path.dirname(os.path.abspath(__file__))
# IMG_DIR = '/'.join(this_file.split('/')[:-2]) + '/img'

COLORS = {
    "red_low": [[0, 100, 100], [10, 255, 255]],
    "red_high": [[160, 100, 100], [179, 255, 255]],
    "orange": [[15, 100, 100], [20, 255, 255]],
    "yellow": [[25, 100, 100], [45, 255, 255]],
    "blue": [[110,30,30], [130,255,255]],
    "green": [[40,60,60], [70,255,255]],
    "black": [[0, 0, 0], [180, 230, 30]],
    "purple": [[140, 100, 100], [160, 255, 255]]
}

def segment_by_color(image, color):
    img = cv2.imread(image)
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
    plt.imshow(mask, cmap='gray')
    plt.title("Segmentation by %s" % color)
    plt.show()

def segment_all(image):
    fig,axes = plt.subplots(nrows = 4, ncols = 2, figsize=(50, 50))
    axes[0, 0].imshow(segment_by_color(image, "red"), cmap='gray')
    axes[0, 0].set_title("Segmentation by red")
    axes[0, 1].imshow(segment_by_color(image, "purple"), cmap='gray')
    axes[0, 1].set_title("Segmentation by purple")
    axes[1, 0].imshow(segment_by_color(image, "black"), cmap='gray')
    axes[1, 0].set_title("Segmentation by black")
    axes[1, 1].imshow(segment_by_color(image, "yellow"), cmap='gray')
    axes[1, 1].set_title("Segmentation by yellow")
    axes[2, 0].imshow(segment_by_color(image, "blue"), cmap='gray')
    axes[2, 0].set_title("Segmentation by blue")
    axes[2, 1].imshow(segment_by_color(image, "orange"), cmap='gray')
    axes[2, 1].set_title("Segmentation by orange")
    axes[3, 0].imshow(segment_by_color(image, "green"), cmap='gray')
    axes[3, 0].set_title("Segmentation by green")
    # for color in COLORS:
    #     mask = segment_by_color(image, color)
    #     axes[0, 0].imshow(mask, cmap='gray')
    #     plt.title("Segmentation by %s" % color)
    plt.show()


if __name__ == '__main__':
    test = './testdata/test.jpg'
    mult = './testdata/multiple_pieces.jpg'
    org_yel = './testdata/org_yel.jpg'
    crossed = './testdata/crossed.jpg'
    cross = './testdata/cross.jpg'
    segment_by_color(cross, "yellow")
    # segment_all('./testdata/test.jpg')
   