#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import cv2


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only

def color_thresh(img, rgb_thresh=(150, 150, 150)):

    # Create an array of zeros same xy size as img, but single channel

    color_select = np.zeros_like(img[:, :, 0])

    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met

    above_thresh = (img[:, :, 0] > rgb_thresh[0]) & (img[:, :, 1]
            > rgb_thresh[1]) & (img[:, :, 2] > rgb_thresh[2])

    # Index the array of zeros with the boolean array and set to 1

    color_select[above_thresh] = 1

    # Return the binary image

    return color_select


# Define a function to convert from image coords to rover coords

def rover_coords(binary_img):

    # Identify nonzero pixels

    (ypos, xpos) = binary_img.nonzero()

    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.

    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)
    return (x_pixel, y_pixel)


# Define a function to convert to radial coords in rover space

def to_polar_coords(x_pixel, y_pixel):

    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel

    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)

    # Calculate angle away from vertical for each pixel

    angles = np.arctan2(y_pixel, x_pixel)
    return (dist, angles)


# Define a function to map rover space pixels to world space

def rotate_pix(xpix, ypix, yaw):

    # Convert yaw to radians

    yaw_rad = yaw * np.pi / 180
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)

    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)

    # Return the result

    return (xpix_rotated, ypix_rotated)


def translate_pix(
    xpix_rot,
    ypix_rot,
    xpos,
    ypos,
    scale,
    ):

    # Apply a scaling and a translation

    xpix_translated = xpix_rot / scale + xpos
    ypix_translated = ypix_rot / scale + ypos

    # Return the result

    return (xpix_translated, ypix_translated)


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work

def pix_to_world(
    xpix,
    ypix,
    xpos,
    ypos,
    yaw,
    world_size,
    scale,
    ):

    # Apply rotation

    (xpix_rot, ypix_rot) = rotate_pix(xpix, ypix, yaw)

    # Apply translation

    (xpix_tran, ypix_tran) = translate_pix(xpix_rot, ypix_rot, xpos,
            ypos, scale)

    # Perform rotation, translation and clipping all at once

    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)

    # Return the result

    return (x_pix_world, y_pix_world)


# this function detects the obstacles by bandbass technique , lower and upper bound for the mountains and unwanted rocks that blocks the road , the band is having brown and dark colors

def find_obstacle(img):

    

    

    # define BGR threshold for dark and bright obstacle colors

    lower_obstacle = np.array([0, 0, 0])
    upper_obstacle = np.array([149, 149, 149])

    # Create a mask. Threshold the HSV image to get only yellow colors

    mask = cv2.inRange(img, lower_obstacle, upper_obstacle)

    # Bitwise-AND mask and original image

    masked_img = cv2.bitwise_and(img, img, mask=mask)

    # threshold the image to generate a binary image

    result = color_thresh(masked_img, (0, 0, 0))

    return result


def find_rock(img):

    # define BGR threshold for dark and bright yellow

    lower_yellow = np.array([14, 128, 128])
    upper_yellow = np.array([34, 255, 255])

    # first we create HSV image version of the original image

    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Create a mask. Threshold the HSV image to get only yellow colors

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Bitwise-AND mask and original image

    masked_img = cv2.bitwise_and(img, img, mask=mask)

    # threshold the image to generate a binary image

    result = color_thresh(masked_img, (0, 0, 0))

    return result

# used to get range of pixels in only range of 6 meters square to increase the fidelity

def range_closure(xpix, ypix, range=55):

    # using distance rule of root x square + y square

    dist = np.sqrt(xpix ** 2 + ypix ** 2)

    # return the pixels in range

    return (xpix[dist < range], ypix[dist < range])


# Define a function to perform a perspective transform

def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image

    return warped


# Apply the above functions in succession and update the Rover state accordingly

def perception_step(Rover):

    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform

    img = Rover.img
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[img.shape[1] / 2 - dst_size,
                             img.shape[0] - bottom_offset],
                             [img.shape[1] / 2 + dst_size, img.shape[0]
                             - bottom_offset], [img.shape[1] / 2
                             + dst_size, img.shape[0] - 2 * dst_size
                             - bottom_offset], [img.shape[1] / 2
                             - dst_size, img.shape[0] - 2 * dst_size
                             - bottom_offset]])

    # 2) Apply perspective transform

    warped = perspect_transform(img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # Get navigable

    navigable = color_thresh(warped)

    # Get obstacles

    obstacles = find_obstacle(warped)

    # identify the rock

    rock_samples = find_rock(warped)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    Rover.vision_image[:, :, 0] = obstacles * 255  # each image is binary image so it has to be multiplied by 255
    Rover.vision_image[:, :, 1] = rock_samples * 255
    Rover.vision_image[:, :, 2] = navigable * 255
    cv2.imshow('warped', warped)
    cv2.imshow('obstacles', obstacles * 255)
    cv2.imshow('rock_samples', rock_samples * 255)
    cv2.imshow('navigable', navigable * 255)
    cv2.imshow('Rover.vision_image', Rover.vision_image)

    # 5) Convert map image pixel values to rover-centric coords

    (Navx, Navy) = rover_coords(navigable)
    (Obsx, Obsy) = rover_coords(obstacles)
    (rokx, roky) = rover_coords(rock_samples)

    # 6) Convert rover-centric pixel values to world coordinates

    (Navx, Navy) = range_closure(Navx, Navy)  # shorten the range of pixels for more fidelity
    (Obsx, Obsy) = range_closure(Obsx, Obsy)  # shorten the range of pixels for more fidelity

        # pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):

    (NavWorldx, NavWorldy) = pix_to_world(
        Navx,
        Navy,
        Rover.pos[0],
        Rover.pos[1],
        Rover.yaw,
        Rover.worldmap.shape[0],
        17,
        )
    (ObsWorldx, ObsWorldy) = pix_to_world(
        Obsx,
        Obsy,
        Rover.pos[0],
        Rover.pos[1],
        Rover.yaw,
        Rover.worldmap.shape[0],
        17,
        )
    (RokWorldx, RokWorldy) = pix_to_world(
        rokx,
        roky,
        Rover.pos[0],
        Rover.pos[1],
        Rover.yaw,
        Rover.worldmap.shape[0],
        17,
        )

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1....
    # update if the Rover is in balanced state, not inclined or rotated around itself in air
    # to check this , check roll and picth of the rover

    if (Rover.pitch < 1 or Rover.pitch > 359) and (Rover.roll < 1
            or Rover.roll > 359):
        Rover.worldmap[ObsWorldy, ObsWorldx, 0] += 1
        Rover.worldmap[RokWorldy, RokWorldx, 1] += 1
        Rover.worldmap[NavWorldy, NavWorldx, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles

    (Navdist, Navangles) = to_polar_coords(Navx, Navy)
    (Rokdist, Rokangles) = to_polar_coords(rokx, roky)
    Rover.nav_dists = Navdist
    Rover.nav_angles = Navangles
    Rover.samples_dists = Rokdist
    Rover.samples_angles = Rokangles
    cv2.waitKey(1)

    return Rover

