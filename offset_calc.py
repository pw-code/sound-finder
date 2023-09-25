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
CAMERA_FOV=25  
# Measured object is 2.1m across, at a distance of 3.8m and fills the screen
# From chatGPT: FOV = 2 * arctan(obj_width/(2*distance))
#   = 2 * arctan(2.1/(2*3.8))
#   = 30.89
# some OV7670 datasheets say 25, and 30 seems not quite right.

SQUARE_SIZE = 24 # match ro calculation logic - roughly match to area of one sample

# Calculate a range of points in 3D space in front of the camera.
# Positions in meters (or factions of) with the camera centered at (0,0,0).
# x=width, y=depth into scene, z=height
# The microphones are spaced around the camera (y=0)

# 90 points calculated from cone.py
sound_points=[(-0.22169466264293988, 1, -0.2955928835239198),
 (-0.22169466264293988, 1, -0.15458685019599186),
 (-0.22169466264293988, 1, -0.013580816868063894),
 (-0.22169466264293988, 1, 0.12742521645986407),
 (-0.22169466264293988, 1, 0.26843124978779204),
 (-0.1423787688959804, 1, -0.2955928835239198),
 (-0.1423787688959804, 1, -0.15458685019599186),
 (-0.1423787688959804, 1, -0.013580816868063894),
 (-0.1423787688959804, 1, 0.12742521645986407),
 (-0.1423787688959804, 1, 0.26843124978779204),
 (-0.06306287514902091, 1, -0.2955928835239198),
 (-0.06306287514902091, 1, -0.15458685019599186),
 (-0.06306287514902091, 1, -0.013580816868063894),
 (-0.06306287514902091, 1, 0.12742521645986407),
 (-0.06306287514902091, 1, 0.26843124978779204),
 (0.016253018597938568, 1, -0.2955928835239198),
 (0.016253018597938568, 1, -0.15458685019599186),
 (0.016253018597938568, 1, -0.013580816868063894),
 (0.016253018597938568, 1, 0.12742521645986407),
 (0.016253018597938568, 1, 0.26843124978779204),
 (0.09556891234489805, 1, -0.2955928835239198),
 (0.09556891234489805, 1, -0.15458685019599186),
 (0.09556891234489805, 1, -0.013580816868063894),
 (0.09556891234489805, 1, 0.12742521645986407),
 (0.09556891234489805, 1, 0.26843124978779204),
 (0.17488480609185753, 1, -0.2955928835239198),
 (0.17488480609185753, 1, -0.15458685019599186),
 (0.17488480609185753, 1, -0.013580816868063894),
 (0.17488480609185753, 1, 0.12742521645986407),
 (0.17488480609185753, 1, 0.26843124978779204),
 (-1.3301679758576392, 6, -1.7735573011435188),
 (-1.3301679758576392, 6, -0.9275211011759511),
 (-1.3301679758576392, 6, -0.08148490120838336),
 (-1.3301679758576392, 6, 0.7645512987591844),
 (-1.3301679758576392, 6, 1.610587498726752),
 (-0.8542726133758825, 6, -1.7735573011435188),
 (-0.8542726133758825, 6, -0.9275211011759511),
 (-0.8542726133758825, 6, -0.08148490120838336),
 (-0.8542726133758825, 6, 0.7645512987591844),
 (-0.8542726133758825, 6, 1.610587498726752),
 (-0.3783772508941256, 6, -1.7735573011435188),
 (-0.3783772508941256, 6, -0.9275211011759511),
 (-0.3783772508941256, 6, -0.08148490120838336),
 (-0.3783772508941256, 6, 0.7645512987591844),
 (-0.3783772508941256, 6, 1.610587498726752),
 (0.09751811158763124, 6, -1.7735573011435188),
 (0.09751811158763124, 6, -0.9275211011759511),
 (0.09751811158763124, 6, -0.08148490120838336),
 (0.09751811158763124, 6, 0.7645512987591844),
 (0.09751811158763124, 6, 1.610587498726752),
 (0.5734134740693881, 6, -1.7735573011435188),
 (0.5734134740693881, 6, -0.9275211011759511),
 (0.5734134740693881, 6, -0.08148490120838336),
 (0.5734134740693881, 6, 0.7645512987591844),
 (0.5734134740693881, 6, 1.610587498726752),
 (1.049308836551145, 6, -1.7735573011435188),
 (1.049308836551145, 6, -0.9275211011759511),
 (1.049308836551145, 6, -0.08148490120838336),
 (1.049308836551145, 6, 0.7645512987591844),
 (1.049308836551145, 6, 1.610587498726752),
 (-2.4386412890723386, 11, -3.251521718763118),
 (-2.4386412890723386, 11, -1.7004553521559103),
 (-2.4386412890723386, 11, -0.14938898554870272),
 (-2.4386412890723386, 11, 1.4016773810585048),
 (-2.4386412890723386, 11, 2.9527437476657123),
 (-1.5661664578557843, 11, -3.251521718763118),
 (-1.5661664578557843, 11, -1.7004553521559103),
 (-1.5661664578557843, 11, -0.14938898554870272),
 (-1.5661664578557843, 11, 1.4016773810585048),
 (-1.5661664578557843, 11, 2.9527437476657123),
 (-0.6936916266392301, 11, -3.251521718763118),
 (-0.6936916266392301, 11, -1.7004553521559103),
 (-0.6936916266392301, 11, -0.14938898554870272),
 (-0.6936916266392301, 11, 1.4016773810585048),
 (-0.6936916266392301, 11, 2.9527437476657123),
 (0.1787832045773241, 11, -3.251521718763118),
 (0.1787832045773241, 11, -1.7004553521559103),
 (0.1787832045773241, 11, -0.14938898554870272),
 (0.1787832045773241, 11, 1.4016773810585048),
 (0.1787832045773241, 11, 2.9527437476657123),
 (1.0512580357938783, 11, -3.251521718763118),
 (1.0512580357938783, 11, -1.7004553521559103),
 (1.0512580357938783, 11, -0.14938898554870272),
 (1.0512580357938783, 11, 1.4016773810585048),
 (1.0512580357938783, 11, 2.9527437476657123),
 (1.9237328670104326, 11, -3.251521718763118),
 (1.9237328670104326, 11, -1.7004553521559103),
 (1.9237328670104326, 11, -0.14938898554870272),
 (1.9237328670104326, 11, 1.4016773810585048),
 (1.9237328670104326, 11, 2.9527437476657123)]

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

offset_x = SQUARE_SIZE
offset_y = SQUARE_SIZE
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
    screen_offsets[p] = (int(x*scale_x + offset_x), int(y*scale_y + offset_y))


###############################################################################################################################

# formatted for C code
print("#define SAMPLE_OFFSET_COUNT " + str(num_points))
print("#define SAMPLE_OFFSET_NUM_CHANNELS " + str(NUM_CHANNELS))
print("#define SAMPLE_OFFSET_HZ " + str(SAMPLE_RATE_HZ))
print("#define SQUARE_SIZE " + str(SQUARE_SIZE))
print()

print("typedef struct screen_point {")
print("    int x, y;")
print("} screen_point_t;")
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
print("const screen_point_t screen_offsets[SAMPLE_OFFSET_COUNT] = {")
for p in range(num_points):
    print("    { " + str(screen_offsets[p][0]) + ", " + str(str(screen_offsets[p][1])), end="")
    if (p+1) >= num_points:
        print(" }")
    else:
        print(" },")

print("};")
print()
