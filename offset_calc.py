#!/usr/bin/env python3
#
# Calculate the expected wave-front time offsets for beam forming
#
import math
import pprint
import numpy as np

NUM_CHANNELS=6
SAMPLE_RATE_HZ=48000 # see audio.h

SPEED_OF_SOUND=343   #343 m / sec

#camera specs
CAMERA_RES_X=320
CAMERA_RES_Y=240
CAMERA_FOV=30  #field of view. Will have to determine this imperically as I don't know the specs
# Measured object is 2.1m across, at a distance of 3.8m and fills the screen
# From chatGPT: FOV = 2 * arctan(obj_width/(2*distance))
#   = 2 * arctan(2.1/(2*3.8))
#   = 30.89

SQUARE_SIZE = 32 #should match drawing code

# Calculate a range of points in 3D space in front of the camera.
# Positions in meters (or factions of) with the camera centered at (0,0,0).
# x=width, y=depth into scene, z=height
# The microphones are spaced around the camera (y=0)

# 64 points calculated from cone.py
sound_points=[(-0.5358983848622454, 2, -0.7145311798163272), (-0.5358983848622454, 2, -0.23817705993877575), (-0.5358983848622454, 2, 0.2381770599387757), (-0.2679491924311227, 2, -0.7145311798163272), (-0.2679491924311227, 2, -0.23817705993877575), (-0.2679491924311227, 2, 0.2381770599387757), (0.0, 2, -0.7145311798163272), (0.0, 2, -0.23817705993877575), (0.0, 2, 0.2381770599387757), (0.2679491924311227, 2, -0.7145311798163272), (0.2679491924311227, 2, -0.23817705993877575), (0.2679491924311227, 2, 0.2381770599387757), (-1.3397459621556136, 5, -1.786327949540818), (-1.3397459621556136, 5, -0.5954426498469394), (-1.3397459621556136, 5, 0.5954426498469392), (-1.3397459621556136, 5, 1.7863279495408177), (-0.6698729810778068, 5, -1.786327949540818), (-0.6698729810778068, 5, -0.5954426498469394), (-0.6698729810778068, 5, 0.5954426498469392), (-0.6698729810778068, 5, 1.7863279495408177), (0.0, 5, -1.786327949540818), (0.0, 5, -0.5954426498469394), (0.0, 5, 0.5954426498469392), (0.0, 5, 1.7863279495408177), (0.6698729810778068, 5, -1.786327949540818), (0.6698729810778068, 5, -0.5954426498469394), (0.6698729810778068, 5, 0.5954426498469392), (0.6698729810778068, 5, 1.7863279495408177), (-2.1435935394489816, 8, -2.8581247192653088), (-2.1435935394489816, 8, -0.952708239755103), (-2.1435935394489816, 8, 0.9527082397551028), (-1.0717967697244908, 8, -2.8581247192653088), (-1.0717967697244908, 8, -0.952708239755103), (-1.0717967697244908, 8, 0.9527082397551028), (0.0, 8, -2.8581247192653088), (0.0, 8, -0.952708239755103), (0.0, 8, 0.9527082397551028), (1.0717967697244908, 8, -2.8581247192653088), (1.0717967697244908, 8, -0.952708239755103), (1.0717967697244908, 8, 0.9527082397551028), (-2.9474411167423495, 11, -3.929921488989799), (-2.9474411167423495, 11, -1.3099738296632664), (-2.9474411167423495, 11, 1.3099738296632664), (-1.4737205583711748, 11, -3.929921488989799), (-1.4737205583711748, 11, -1.3099738296632664), (-1.4737205583711748, 11, 1.3099738296632664), (0.0, 11, -3.929921488989799), (0.0, 11, -1.3099738296632664), (0.0, 11, 1.3099738296632664), (1.4737205583711748, 11, -3.929921488989799), (1.4737205583711748, 11, -1.3099738296632664), (1.4737205583711748, 11, 1.3099738296632664), (-3.7512886940357175, 14, -5.001718258714289), (-3.7512886940357175, 14, -1.66723941957143), (-3.7512886940357175, 14, 1.6672394195714295), (-1.8756443470178588, 14, -5.001718258714289), (-1.8756443470178588, 14, -1.66723941957143), (-1.8756443470178588, 14, 1.6672394195714295), (0.0, 14, -5.001718258714289), (0.0, 14, -1.66723941957143), (0.0, 14, 1.6672394195714295), (1.8756443470178588, 14, -5.001718258714289), (1.8756443470178588, 14, -1.66723941957143), (1.8756443470178588, 14, 1.6672394195714295)]


###############################################################################################################################

# mic offsets from (0,0,0) at center of the mic array
mic_offsets=[(0,0,0) for c in range(NUM_CHANNELS)]

mic_array_radius=0.105 #10.5cm in meters
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

distances=set(map(lambda p: p[1], sound_points))

def calc_distance_size(d):
    points=list(filter(lambda p: p[1]==d, sound_points))
    minx = min(map(lambda p: p[0], points))
    maxx = max(map(lambda p: p[0], points))
    minz = min(map(lambda p: p[2], points))
    maxz = max(map(lambda p: p[2], points))
    rangex = maxx-minx
    rangez = maxz-minz
    return [(minx, minz), (rangex, rangez)]

size_by_distance={d: calc_distance_size(d) for d in distances}

offset_x = (SQUARE_SIZE * 1.5)
offset_y = (SQUARE_SIZE * 1.5)
scale_x = CAMERA_RES_X - 2*offset_x
scale_y = CAMERA_RES_Y - 2*offset_y

# Convert 3D points into 2D screen space
# screen_offsets[point] is audio sample display position on screen
screen_offsets=[(0,0) for i in range(num_points)]
for p in range(num_points):
    point = sound_points[p]
    size = size_by_distance[point[1]]
    #scale this point within it's size range, then scale to screen size
    x = (point[0] - size[0][0]) / size[1][0]
    y = (point[2] - size[0][1]) / size[1][1]
    screen_offsets[p] = (int(x*scale_x)+offset_x, int(y*scale_y)+offset_y)


###############################################################################################################################

# formatted for C code
print("#define SAMPLE_OFFSET_COUNT " + str(num_points))
print("#define SAMPLE_OFFSET_NUM_CHANNELS " + str(NUM_CHANNELS))
print("#define SAMPLE_OFFSET_HZ " + str(SAMPLE_RATE_HZ))
print("#define SQUARE_SIZE " + str(SQUARE_SIZE))
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
