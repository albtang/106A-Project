#!/usr/bin/env python
"""Skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Amay Saxena

A test script for pointcloud projection.
"""
import os
import numpy as np
import ros_numpy
from pointcloud_segmentation import project_points
from matplotlib import pyplot as plt

this_file = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = '/'.join(this_file.split('/')[:]) + '/testdata'

def main():
    IDX2D = lambda i, j, dj: dj * i + j

    image = np.load(IMG_DIR + '/rgb0.npy')
    cam_matrix = np.load(IMG_DIR + '/cam_info0.npy')
    points = ros_numpy.point_cloud2.split_rgb_field(np.load(IMG_DIR + '/points0.npy'))

    rot = np.array([[0.9999215006828308, 0.010321159847080708, -0.007101841736584902], 
                    [-0.010323554277420044, 0.999946653842926, -0.0003005565085913986], 
                    [0.007098360452800989, 0.0003738491504918784, 0.9999747276306152]])
    
    trans = np.array([0.014698210172355175, 0.00022185585112310946, 0.0003382108989171684])

    projected_image = np.zeros(image.shape, dtype=np.int32)

    xyz = np.vstack((points['x'], points['y'], points['z']))
    rgb = np.vstack((points['r'], points['g'], points['b']))

    pixel_coords = project_points(xyz, cam_matrix, trans, rot)

    image_h, image_w = image.shape[:2]

    in_frame = ((0 <= pixel_coords[0]) & (pixel_coords[0] < image_w)
                & (0 <= pixel_coords[1]) & (pixel_coords[1] < image_h))
    
    pixel_coords = pixel_coords[:, in_frame]
    j, i = pixel_coords
    linearized_pixel_coords = IDX2D(i, j, image.shape[1])

    lin_proj_image = projected_image.reshape(-1, 3)
    
    lin_proj_image[linearized_pixel_coords, :] = rgb.T[in_frame, :]

    projected_image = lin_proj_image.reshape(projected_image.shape)

    fig=plt.figure(figsize=(15, 30))
    fig.add_subplot(121)
    plt.imshow(image, interpolation='nearest')
    fig.add_subplot(122)
    plt.imshow(projected_image, interpolation='nearest')
    plt.show()

if __name__ == '__main__':
    main()
