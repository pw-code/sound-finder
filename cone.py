import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calculate_rectangle_width(d, fov_degrees):
    fov_radians = math.radians(fov_degrees)  # Convert FOV from degrees to radians
    width = 2 * d * math.tan(fov_radians / 2)
    return width

def calculate_rectangle(d, fov_degrees, ratio):
    w = calculate_rectangle_width(d, fov_degrees)
    h = w * ratio
    return (w,h)

def frange(x, y, step):
    while x < y:
        yield x
        x += step

def make_points(num_points, points, distance, fov, ratio):
    # rectangle at that distance is a specific size, that we need to cover with points
    rect = calculate_rectangle(distance, fov, ratio)

    # how may points to allocate to x,y directions
    # px = py = math.sqrt(num_points)
    # while (px/py) > ratio:
    #     px += px * 1.1
    #     py -= py * 1.1
    grid_width = math.sqrt(num_points * ratio)
    grid_height = num_points / grid_width
    step_x = rect[0] / grid_width
    step_y = rect[1] / grid_height

    for x in frange(-rect[0]/2, rect[0]/2, step_x):
        for y in frange(-rect[1]/2, rect[1]/2, step_y):
            points.append((x,distance,y))

def generate_cone_points(num_points, closest_distance, far_distance, distance_step, fov, ratio):
    points = []
    points_per_layer = num_points / (far_distance - closest_distance + distance_step) * distance_step
    for d in range(closest_distance, far_distance + 1, distance_step):
        make_points(points_per_layer, points, d, fov, ratio)
    return points

ratio = 4/3 # 4:3 resolution ratio
fov = 30    # field of view of camera (from offset_calc.py)
num_points = 64 # enough for detail without being too too slow to sample
cone_points = generate_cone_points(num_points, 2, 15, 3, fov, ratio)

# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(*zip(*cone_points), s=10, c='b')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title(str(len(cone_points)) + ' Equally Spaced Points')

plt.show()

print("# Sound points to analyse (x,y,z)")
print("# Place this in offset_calc.py")
print("# " + str(len(cone_points)) + " points calculated from cone.py")
print("sound_points=", end="")
print(cone_points)

