import numpy as np
import cv2


def generic_threshold(img, lower_thresh=(160, 160, 160), upper_thresh=(255, 255, 255)):
    """
    Color thresholding function
    :param upper_thresh: Upper color threshold
    :param lower_thresh: Lower color threshold
    :param img: The image to be thresholded
    :return: Binary thresholded image
    """
    color_select = np.zeros_like(img[:, :, 0])
    thresh_condition = (
            (img[:, :, 0] > lower_thresh[0])
            & (img[:, :, 1] > lower_thresh[1])
            & (img[:, :, 2] > lower_thresh[2])
            & (img[:, :, 0] < upper_thresh[0])
            & (img[:, :, 1] < upper_thresh[1])
            & (img[:, :, 2] < upper_thresh[2])
    )
    color_select[thresh_condition] = 1
    return color_select


def rover_coords(binary_img):
    """
    Convert image coordinates to rover coordinates
    :param binary_img: Thresholded image
    :return: Rover coordinates
    """
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)
    return x_pixel, y_pixel


def to_polar_coords(x_pixel, y_pixel):
    """
    Convert (x_pixel, y_pixel) to (distance, angle) in polar coordinates in rover space
    :param x_pixel: X
    :param y_pixel: Y
    :return: distances, angles
    """
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


def rotate_pix(xpix, ypix, yaw):
    """
    Map rover space pixels to world space
    :param xpix: X of pixels
    :param ypix: Y of pixels
    :param yaw: yaw
    :return: x_rotated, y_rotated
    """
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Rotate
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    """
    Apply transformation
    :param xpix_rot: Rotated X
    :param ypix_rot: Rotated Y
    :param xpos: X position of Rover
    :param ypos: Y position of Rover
    :param scale: scaling factor
    :return: X translated, Y translated
    """
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    return xpix_translated, ypix_translated


# Define a function to
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    """
    Apply rotation and translation (and clipping)
    :param xpix: X of pixels
    :param ypix: Y of pixels
    :param xpos: X position of Rover
    :param ypos: Y position of Rover
    :param yaw: Turning angle
    :param world_size: Size of traversable map
    :param scale: scaling factor
    :return: X world, Y world
    """
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    return x_pix_world, y_pix_world


def perspect_transform(img, src, dst):
    """
    Perform perspective transform
    :param img: Image
    :param src: Matrix defining calibration grid
    :param dst: Define rover position
    :return:
    """
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:, :, 0]), M, (img.shape[1], img.shape[0]))
    return warped, mask


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    """
    Perform perception step for Rover
    :param Rover: Rover class defining current state of the rover
    :return: Updated Rover object
    """
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    # Source is found by manual inspection of calibration image
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              [Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              ])
    # 2) Apply perspective transform
    warped, mask = perspect_transform(Rover.img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    thresholded = generic_threshold(warped, (160, 160, 160))
    wall_thresh = generic_threshold(warped, (0, 0, 0), (160, 160, 160))
    obstacle_map = np.absolute(np.float32(thresholded) - 1) * mask
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:, :, 0] = obstacle_map * 255
    Rover.vision_image[:, :, 2] = thresholded * 255
    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(thresholded)
    xpix_walls, ypix_walls = rover_coords(wall_thresh)
    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    x_world, y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1],
                                    Rover.yaw, world_size, scale)
    obstacle_x, obstacle_y = rover_coords(obstacle_map)
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_x, obstacle_y, Rover.pos[0], Rover.pos[1], Rover.yaw,
                                                      world_size, scale)
    xpix_world_walls, ypix_world_walls = pix_to_world(xpix_walls, ypix_walls, Rover.pos[0], Rover.pos[1], Rover.yaw,
                                                      world_size, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[y_world, x_world, 2] += 255
    Rover.worldmap[y_world, x_world, 0] -= 100
    Rover.worldmap[ypix_world_walls, xpix_world_walls, 2] -= 10
    Rover.worldmap[ypix_world_walls, xpix_world_walls, 0] += 255
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 2] -= 0
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 160
    # 8) Convert rover-centric pixel positions to polar coordinates
    dist, angles = to_polar_coords(xpix, ypix)
    # Update Rover pixel distances and angles
    Rover.nav_angles = angles
    Rover.nav_dists = dist

    # Find rocks
    rock_map = generic_threshold(warped, (150, 100, 0), (255, 255, 50))
    if rock_map.any():
        rock_x, rock_y = rover_coords(rock_map)
        rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, Rover.pos[0], Rover.pos[1], Rover.yaw,
                                                  world_size, scale)
        rock_dist, rock_ang = to_polar_coords(rock_x, rock_y)
        rock_idx = np.argmin(rock_dist)
        rock_x_center = rock_x_world[rock_idx]
        rock_y_center = rock_y_world[rock_idx]
        Rover.worldmap[rock_y_center, rock_x_center, 1] = 255
        Rover.vision_image[:, :, 1] = rock_map * 255
        Rover.rock_angle = rock_ang
    else:
        Rover.vision_image[:, :, 1] = 0

    return Rover
