import numpy as np
import math

# Given a float representing the start position, end position, and distance between scans, all in radians
# And given a float32[] of ranges, convert the ranges into lines with angles.
a = np.array([1,2, 1.5], dtype='f')
current_pos = [0, 0]

start_angle = 0 # In radians
angle_step = 90 * math.pi / 180 # In radians (90 degrees)

def approach1(start_angle, current_pos, distances, angle_step):
    # Swap this to transform the numpy array - it should be faster
    for i,r in enumerate(distances):
        x = current_pos[0] + (r * math.cos(start_angle + angle_step * i))
        y = current_pos[1] + (r * math.sin(start_angle + angle_step * i))
        print("Moving to angle {0}".format(angle_step * i * 180 / math.pi))
        print(str(x) + ", " + str(y) + "\n")

def approach2(start_angle, current_pos, distances, angle_step):
    angles = start_angle + np.arange(len(distances)) * angle_step
        
    # Vectorized transformation
    x = current_pos[0] + distances * np.cos(angles)
    y = current_pos[1] + distances * np.sin(angles)
                        
    return np.column_stack((x, y))

approach1(start_angle, current_pos, a, angle_step)
print(approach2(start_angle, current_pos, a, angle_step))


"""
When filling this into a ros node, make sure to first filter points that are too close/far based off the range_min/range_max lidar parameters. 
Swap the angles array to go from start -> end on step size of angle_increment parameter.
Converting to xy points should be the exact same
"""

