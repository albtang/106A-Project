#!/usr/bin/env python
"""Skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Amay Saxena

This file contains skeleton code for performing point-cloud projection
and segmentation.

Once you are confident in your implementation in image_segmentation.py and
pointcloud_segmentation.py, run main.py to begin publishing a segmented
pointcloud.
"""
import numpy as np

def segment_pointcloud(points, segmented_image, cam_matrix, trans, rot):
    IDX2D = lambda i, j, dj: dj * i + j
    xyz = np.vstack((points['x'], points['y'], points['z']))
    pixel_coords = project_points(xyz, cam_matrix, trans, rot)

    image_h, image_w = segmented_image.shape[:2]

    in_frame = ((0 <= pixel_coords[0]) & (pixel_coords[0] < image_w)
                & (0 <= pixel_coords[1]) & (pixel_coords[1] < image_h))
    
    points = points[in_frame]
    pixel_coords = pixel_coords[:, in_frame]
    j, i = pixel_coords
    linearized_pixel_coords = IDX2D(i, j, segmented_image.shape[1])
    linearized_segmentation = segmented_image.reshape(-1)
    point_labels = linearized_segmentation[linearized_pixel_coords]
    return points[point_labels == 1]

def project_points(points, cam_matrix, trans, rot):
    """
    This funtion should perform the job of projecting the input pointcloud onto the frame
    of an image captured by a camera with camera matrix as given, of dimensions as given,
    in pixels.

    points is an 3 x N array where the ith entry is an (x, y, z) point in 3D space, in 
    the reference frame of the depth camera. This corresponds to the tf frame
    camera_depth_optical_frame. However, the image is taken by an RGB camera, with
    reference frame camera_color_optical_frame. (trans, rot) together give the translation
    vector and rotation matrix that transform points in the depth camera frame to points
    in the RGB camera frame.

    For each point in points, compute the pixel co-ordinates (u, v) onto which that point
    would be projected.

    This function should return a 2 x N integer array of pixel co-ordinates. The ith entry 
    should  be the index (u, v) of the pixel onto which the ith point in the pointcloud should 
    get projected.

    Use the point projection model introduced in the lab documentation to perform this
    projection.

    Note that this function should be able to operate on large pointclouds very efficiently.
    Make good use of numpy functions to vectorize and to act on the entire pointcloud at once.

    Hint 1: The staff solution uses no loops, and is fewer than 5 lines long.

    Hint 2: You will need to first transform the pointcloud into the reference frame of the
    RGB camera, before you can use the camera matrix to perform the projection. Don't use
    a loop to do this. Instead, try to see if you can act on the entire pointcloud at once
    using a single matrix multiplication and numpy functions.

    Hint 3: We learned in the lab doc that to project a single point onto the image frame we
    can multiply the intrinsic matrix with the column vector of the point. To poject the whole
    cloud, then, we could just go in a for loop and multiply each point by the camera matrix
    one by one. This will be slow, because python for loops are expensive. Is there a way to
    multiply all points with the cam matrix without looping over them, in just one matrix
    multiplication?

    Args:
    
    points: (numpy.ndarray) Array of shape (3, N). ith entry is a 3D array representing
            a single (x, y, z) point in the reference frame of the camera.

    cam_matrix: (numpy.ndarray) Array of shape (3, 3) representing the camera intrinsic
                matrix.

                This parameter takes the standard form of the camera matrix as described
                in the lab doc:

                [[fx, s,  x0],
                 [0,  fy, y0],
                 [0,  0,  1 ]]

    trans: (numpy.ndarray) 1D array of length 3. This is the translation vector that
    offsets points in the depth camera reference frame to the RGB camera reference frame.

    rot: (numpy.ndarray) array of shape (3, 3). This is the 3x3 rotation matrix that takes
    points from the depth camera frame to the RGB camera frame.

    """

    # STEP 1: Transform pointcloud into new reference frame.
    points = np.dot(rot, points) + trans[:, None]

    # STEP 2: Project new pointcloud onto image frame using K matrix.
    # gives a 3 x N array of image plane coordinates in homogenous coordinates.
    homo_pixel_coords = np.dot(cam_matrix, points)

    # STEP 3: Convert homogenous coordinates to regular 2D coordinates.
    # To do this, you need to divide the first two coordinates of homo_pixel_coords
    # by the third coordinate.
    pixel_coords = homo_pixel_coords[0:2] / homo_pixel_coords[2]

    # STEP 4: Convert to integers. Take the floor of pixel_coords then cast it
    # to an integer type, like numpy.int32
    pixel_coords = np.int32(np.floor(pixel_coords))

    return pixel_coords
