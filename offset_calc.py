#!/usr/bin/env python3
#
# Calculate the expected wave-front time offsets for beam forming
#
import math
import pprint

# 1 mitre chunks, in a grid pattern (x wide, and y deep (from sensors))
# Flattened field of view in front of sensors. Sensor center is (0,0,0)
# Actual positons will be centered on (0,0,0). e.g. Width 10 is: -5 <= x <= 5

WIDTH=10        # x axis
DEPTH=10        # y axis
HEIGHT=3        # z axis

NUM_CHANNELS=6
SAMPLE_RATE_HZ=30000 # see main.c

SPEED_OF_SOUND=343   #343 m / sec


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

# offsets[x][y][z][channel] is audio sample# offset for each channel at each position.
offsets=[[[[0
            for c in range(NUM_CHANNELS)]
                for z in range(HEIGHT)] 
                    for y in range(DEPTH)] 
                        for x in range(WIDTH)]

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


for z_whole in range(HEIGHT):
    z = z_whole - (HEIGHT/2) #center on 0 +/- for calculations
    
    for y_whole in range(DEPTH):
        y = y_whole #ranges from 0 outwards

        for x_whole in range(WIDTH):
            x = x_whole - (WIDTH/2) #center on 0 +/- for calculations

            # for each mic/channel, calculate the expected audio delay from source(x,y,z) to mic(x,y,z)
            for channel in range(NUM_CHANNELS):
                o = calculate_delay_offset((x,y,z), mic_offsets[channel])
                offsets[x_whole][y_whole][z_whole][channel] = int(o) #fixed index offsets


pprint.pprint(offsets)

# formatted for C code
print("#define SAMPLE_OFFSET_WIDTH " + str(WIDTH) + " /* x axis */")
print("#define SAMPLE_OFFSET_DEPTH " + str(DEPTH) + " /* y axis */")
print("#define SAMPLE_OFFSET_HEIGHT " + str(HEIGHT) + " /* z axis */")
print("#define SAMPLE_OFFSET_NUM_CHANNELS " + str(NUM_CHANNELS))
print("#define SAMPLE_OFFSET_HZ " + str(SAMPLE_RATE_HZ))
print()
print("/* Sample Offsets[x,y,z,chan] */")
print("const int sample_offsets[SAMPLE_OFFSET_WIDTH][SAMPLE_OFFSET_DEPTH][SAMPLE_OFFSET_HEIGHT][SAMPLE_OFFSET_NUM_CHANNELS] = {")

def comma_end(n, limit):
    if (n+1) < limit:
        return ","
    else:
        return ""

for x in range(WIDTH):

    print("  {")
    for y in range(DEPTH):

        print("    {")
        for z in range(HEIGHT):

            print("      {", end="")
            for channel in range(NUM_CHANNELS):
                print(" " + str(offsets[x][y][z][channel]) + comma_end(channel, NUM_CHANNELS), end="")

            print(" }" + comma_end(z, HEIGHT))

        print("    }" + comma_end(y, DEPTH))

    print("  }" + comma_end(x, WIDTH))

print("};")
