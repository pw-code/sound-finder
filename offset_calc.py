#!/usr/bin/env python3
#
# Calculate the expected wave-front time offsets for beam forming
#
import math
import pprint
import numpy as np

NUM_CHANNELS=6
SAMPLE_RATE_HZ=44100 # see main.c

SPEED_OF_SOUND=343   #343 m / sec

#camera specs
CAMERA_RES_X=320
CAMERA_RES_Y=240
CAMERA_FOV=25  #field of view. Will have to determine this imperically as I don't know the specs


# Calculate a range of points in 3D space in front of the camera.
# Positions in meters (or factions of) with the camera centered at (0,0,0).
# x=width, y=depth into scene, z=height
# The microphones are spaced around the camera (y=0)

# 84 points calculated from cone.py
sound_points=[(-0.9326153163099972, 2, -1.2434870884133296), (-0.9326153163099972, 2, -0.20807717132326342), (-0.9326153163099972, 2, 0.8273327457668027), (-0.350197237946835, 2, -1.2434870884133296), (-0.350197237946835, 2, -0.20807717132326342), (-0.350197237946835, 2, 0.8273327457668027), (0.23222084041632718, 2, -1.2434870884133296), (0.23222084041632718, 2, -0.20807717132326342), (0.23222084041632718, 2, 0.8273327457668027), (0.8146389187794894, 2, -1.2434870884133296), (0.8146389187794894, 2, -0.20807717132326342), (0.8146389187794894, 2, 0.8273327457668027), (-1.8652306326199943, 4, -2.486974176826659), (-1.8652306326199943, 4, -0.41615434264652684), (-1.8652306326199943, 4, 1.6546654915336054), (-0.70039447589367, 4, -2.486974176826659), (-0.70039447589367, 4, -0.41615434264652684), (-0.70039447589367, 4, 1.6546654915336054), (0.46444168083265436, 4, -2.486974176826659), (0.46444168083265436, 4, -0.41615434264652684), (0.46444168083265436, 4, 1.6546654915336054), (1.6292778375589787, 4, -2.486974176826659), (1.6292778375589787, 4, -0.41615434264652684), (1.6292778375589787, 4, 1.6546654915336054), (-2.7978459489299916, 6, -3.7304612652399887), (-2.7978459489299916, 6, -0.6242315139697907), (-2.7978459489299916, 6, 2.4819982373004073), (-1.0505917138405052, 6, -3.7304612652399887), (-1.0505917138405052, 6, -0.6242315139697907), (-1.0505917138405052, 6, 2.4819982373004073), (0.6966625212489812, 6, -3.7304612652399887), (0.6966625212489812, 6, -0.6242315139697907), (0.6966625212489812, 6, 2.4819982373004073), (2.4439167563384676, 6, -3.7304612652399887), (2.4439167563384676, 6, -0.6242315139697907), (2.4439167563384676, 6, 2.4819982373004073), (-3.7304612652399887, 8, -4.973948353653318), (-3.7304612652399887, 8, -0.8323086852930537), (-3.7304612652399887, 8, 3.309330983067211), (-1.40078895178734, 8, -4.973948353653318), (-1.40078895178734, 8, -0.8323086852930537), (-1.40078895178734, 8, 3.309330983067211), (0.9288833616653087, 8, -4.973948353653318), (0.9288833616653087, 8, -0.8323086852930537), (0.9288833616653087, 8, 3.309330983067211), (3.2585556751179574, 8, -4.973948353653318), (3.2585556751179574, 8, -0.8323086852930537), (3.2585556751179574, 8, 3.309330983067211), (-4.663076581549986, 10, -6.217435442066648), (-4.663076581549986, 10, -1.040385856616318), (-4.663076581549986, 10, 4.136663728834012), (-1.7509861897341752, 10, -6.217435442066648), (-1.7509861897341752, 10, -1.040385856616318), (-1.7509861897341752, 10, 4.136663728834012), (1.1611042020816358, 10, -6.217435442066648), (1.1611042020816358, 10, -1.040385856616318), (1.1611042020816358, 10, 4.136663728834012), (4.073194593897447, 10, -6.217435442066648), (4.073194593897447, 10, -1.040385856616318), (4.073194593897447, 10, 4.136663728834012), (-5.595691897859983, 12, -7.460922530479977), (-5.595691897859983, 12, -1.2484630279395814), (-5.595691897859983, 12, 4.9639964746008145), (-2.1011834276810104, 12, -7.460922530479977), (-2.1011834276810104, 12, -1.2484630279395814), (-2.1011834276810104, 12, 4.9639964746008145), (1.3933250424979624, 12, -7.460922530479977), (1.3933250424979624, 12, -1.2484630279395814), (1.3933250424979624, 12, 4.9639964746008145), (4.887833512676935, 12, -7.460922530479977), (4.887833512676935, 12, -1.2484630279395814), (4.887833512676935, 12, 4.9639964746008145), (-6.52830721416998, 14, -8.704409618893306), (-6.52830721416998, 14, -1.4565401992628448), (-6.52830721416998, 14, 5.791329220367617), (-2.4513806656278456, 14, -8.704409618893306), (-2.4513806656278456, 14, -1.4565401992628448), (-2.4513806656278456, 14, 5.791329220367617), (1.625545882914289, 14, -8.704409618893306), (1.625545882914289, 14, -1.4565401992628448), (1.625545882914289, 14, 5.791329220367617), (5.702472431456424, 14, -8.704409618893306), (5.702472431456424, 14, -1.4565401992628448), (5.702472431456424, 14, 5.791329220367617)]


###############################################################################################################################

# mic offsets from (0,0,0) at center of the mic array
mic_offsets=[(0,0,0) for c in range(NUM_CHANNELS)]

mic_array_radius=0.12 #12cm in meters
for m in range(NUM_CHANNELS):
    angle = (math.pi * 2) / NUM_CHANNELS * m   #divide 2PI radius (circle) into chunks
    x = math.cos(angle) * mic_array_radius
    y = 0 # at sensors position
    z = math.sin(angle) * mic_array_radius
    mic_offsets[m] = (x,y,z)
print("/* Mic offsets:")
pprint.pprint(mic_offsets)
print("*/")

###############################################################################################################################

# Calculate the distance between two points in 3D
def distance(pos_a: tuple[float,float,float], pos_b: tuple[float,float,float]) -> float:
    x1 = pos_a[0]
    y1 = pos_a[1]
    z1 = pos_a[2]
    x2 = pos_b[0]
    y2 = pos_b[1]
    z2 = pos_b[2]
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)


# Calculate the delay of a signal reaching this mic from a postion (compared to a mic at (0,0,0))
def calculate_delay_offset(sound_pos: tuple[float,float,float], mic_pos: tuple[float,float,float]) -> float:
    zero_dist = distance((0,0,0), sound_pos)
    mic_dist = distance(mic_pos, sound_pos)

    # the delay for this mic is the distance difference compared with the speed of sound
    delta_dist = zero_dist - mic_dist
    delta_seconds = delta_dist / SPEED_OF_SOUND
    #how many samples is this?
    num_samples = delta_seconds * SAMPLE_RATE_HZ
    return num_samples


# sample_offsets[point][channel] is audio sample# offset for each channel at each position.
num_points=len(sound_points)
sample_offsets=[[0 for c in range(NUM_CHANNELS)] for i in range(num_points)]

for p in range(num_points):
    # for each mic/channel, calculate the expected audio delay from source(x,y,z) to mic(x,y,z)
    for channel in range(NUM_CHANNELS):
        o = calculate_delay_offset(sound_points[p], mic_offsets[channel])
        sample_offsets[p][channel] = int(o) #fixed index offsets


###############################################################################################################################

def project_3d_to_2d(points_3d, intrinsic_matrix, extrinsic_matrix, viewport):
    """
    Project 3D world coordinates into 2D screen coordinates.

    :param points_3d: Array of 3D world coordinates (shape: N x 3).
    :param intrinsic_matrix: 3x3 camera intrinsic matrix.
    :param extrinsic_matrix: 4x4 camera extrinsic matrix.
    :param viewport: Tuple (width, height) of the viewport dimensions.
    :return: Array of 2D screen coordinates (shape: N x 2).
    """
    # Add homogeneous coordinates to 3D points
    ones = np.ones((points_3d.shape[0], 1))
    points_3d_homo = np.hstack((points_3d, ones))

    # Apply extrinsic matrix
    transformed_points = np.dot(extrinsic_matrix, points_3d_homo.T).T

    # Apply intrinsic matrix
    projected_points = np.dot(intrinsic_matrix, transformed_points[:, :3].T).T

    # Normalize homogeneous coordinates
    projected_points /= projected_points[:, 2:3]

    # Map to viewport coordinates
    x = projected_points[:, 0]
    y = projected_points[:, 1]
    x_min, y_min = viewport
    x_max = x_min + viewport[0]
    y_max = y_min + viewport[1]
    mapped_x = (x + 1) * 0.5 * (x_max - x_min) + x_min
    mapped_y = (1 - y) * 0.5 * (y_max - y_min) + y_min

    return np.column_stack((mapped_x, mapped_y))

def rotation_matrix_from_euler(yaw, pitch, roll):
    """
    Calculate the 3x3 rotation matrix from yaw, pitch, and roll angles.

    :param yaw: Yaw angle (rotation around vertical axis, in radians).
    :param pitch: Pitch angle (rotation around lateral axis, in radians).
    :param roll: Roll angle (rotation around longitudinal axis, in radians).
    :return: 3x3 rotation matrix.
    """
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    rotation_matrix = np.dot(R_z, np.dot(R_y, R_x))
    return rotation_matrix

def extrinsic_matrix_from_position_and_angles(position, yaw, pitch, roll):
    """
    Calculate the 4x4 extrinsic matrix from position and rotation angles.

    :param position: 3-element array representing position (x, y, z).
    :param yaw: Yaw angle (rotation around vertical axis, in radians).
    :param pitch: Pitch angle (rotation around lateral axis, in radians).
    :param roll: Roll angle (rotation around longitudinal axis, in radians).
    :return: 4x4 extrinsic matrix.
    """
    rotation_matrix = rotation_matrix_from_euler(yaw, pitch, roll)
    extrinsic_matrix = np.eye(4)
    extrinsic_matrix[:3, :3] = rotation_matrix
    extrinsic_matrix[:3, 3] = position
    return extrinsic_matrix

# Example position and angles
camera_position = np.array([0, 0, 0])
yaw = np.radians(0)
pitch = np.radians(0)
roll = np.radians(0)

# Calculate the extrinsic matrix (camera pose)
extrinsic_matrix = extrinsic_matrix_from_position_and_angles(camera_position, yaw, pitch, roll)


def intrinsic_matrix_from_fov(image_width, image_height, fov_degrees):
    """
    Calculate the 3x3 intrinsic matrix from camera FOV.

    :param image_width: Width of the image in pixels.
    :param image_height: Height of the image in pixels.
    :param fov_degrees: Horizontal field of view in degrees.
    :return: 3x3 intrinsic matrix.
    """
    fov_radians = np.radians(fov_degrees)
    focal_length = (image_width / 2) / np.tan(fov_radians / 2)
    cx = image_width / 2
    cy = image_height / 2

    intrinsic_matrix = np.array([[focal_length, 0, cx],
                                 [0, focal_length, cy],
                                 [0, 0, 1]])
    return intrinsic_matrix

# Example intrinsic matrix (camera calibration)
intrinsic_matrix = intrinsic_matrix_from_fov(CAMERA_RES_X, CAMERA_RES_Y, CAMERA_FOV)


# Convert 3D points into 2D screen space
# screen_offsets[point] is audio sample display position on screen
#screen_offsets=[(0,0) for i in range(num_points)]
# for p in range(num_points):
#     # onscreen.x = ((camera.zfar / vertex.in_view.z) * vertex.in_view.x - camera.fov) + screen.width / 2.0;
#     # onscreen.y = ((scene.zfar / vertex.in_view.z) * vertex.in_view.y - camera.fov) + screen.height / 2.0;
#     x = ((camera.zfar / vertex.in_view.z) * vertex.in_view.x - camera.fov) + screen.width / 2.0;
#     y = ((scene.zfar / vertex.in_view.z) * vertex.in_view.y - camera.fov) + screen.height / 2.0;
#     screen_offsets[p] = (x,y)
# Project 3D points to 2D screen coordinates
viewport = (CAMERA_RES_X, CAMERA_RES_Y)
screen_offsets = project_3d_to_2d(np.array(sound_points), intrinsic_matrix, extrinsic_matrix, viewport)


###############################################################################################################################

# formatted for C code
print("#define SAMPLE_OFFSET_COUNT " + str(num_points))
print("#define SAMPLE_OFFSET_NUM_CHANNELS " + str(NUM_CHANNELS))
print("#define SAMPLE_OFFSET_HZ " + str(SAMPLE_RATE_HZ))
print()

print("/* Sample Offsets[point,chan] */")
print("const int sample_offsets[SAMPLE_OFFSET_COUNT][SAMPLE_OFFSET_NUM_CHANNELS] = {")
for p in range(num_points):
    print("    { ", end="")
    for channel in range(NUM_CHANNELS):
        if channel > 0:
            print(", ", end="")
        print(str(sample_offsets[p][channel]), end="")

    if (p+1) >= num_points:
        print(" }")
    else:
        print(" },")

print("};")
print()

print("/* Screen Offsets[point,chan] */")
print("typedef struct screen_point {")
print("    int x, y;")
print("} screen_point_t;")
print("const screen_point_t screen_offsets[SAMPLE_OFFSET_COUNT] = {")
for p in range(num_points):
    print("    { " + str(screen_offsets[p][0]) + ", " + str(str(screen_offsets[p][1])), end="")
    if (p+1) >= num_points:
        print(" }")
    else:
        print(" },")

print("};")
print()
