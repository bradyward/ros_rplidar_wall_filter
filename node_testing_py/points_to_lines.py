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
        
    # Turn polar  to cartesian
    x = distances * np.cos(angles)
    y = distances * np.sin(angles)

    """ Transformation instructions """
    # Rotate using the transform library
    ## Get cos and sin of robot pose, then apply to a rotation matrix: 
    ## [x]  [cos, -sin]
    ## [y]  [sin, cos ]

    ## Get the robot pose with a dedicated "transform listener". It pulls data from the tf node and gives us a transform by calling methods of the tf2_ros python library. Get the angle things for transforming by doing a bunch of math with quaternians. That ends with a final theta that we use in the transformation matrix above

    # Add robot pose to x and y

    # Check all points are in bounds and trim

    # Publish
                        
    return np.column_stack((x, y))

##approach1(start_angle, current_pos, a, angle_step)
#print(approach2(start_angle, current_pos, a, angle_step))


"""
When filling this into a ros node, make sure to first filter points that are too close/far based off the range_min/range_max lidar parameters. 
Swap the angles array to go from start -> end on step size of angle_increment parameter.
Converting to xy points should be the exact same
"""

def numpy_quat_to_theta(q):
    w=q[0]
    x=q[1]
    y=q[2]
    z=q[3]

    #q = np.array([[0, -quat['z'], quat['y']], [quat['z'], 0, -quat['x']], [-quat['y'], quat['x'], 0]])
    #return np.eye(3) + 2*quat['w']*q + 2*q*q # source code says 2*q@q ??? https://cookierobotics.com/080/

    #sinp = np.sqrt(1 + 2 * (q[0] * q[2] - q[1] * q[3]))
    #cosp = np.sqrt(1 - 2 * (q[0] * q[2] - q[1] * q[3]))
    #return 2 * np.arctan(sinp, cosp) - math.pi/2

    #return math.asin(2 * ((q[1] * q[3]) - (q[0] * q[2])))
    #return math.asin(2 * ((q[0] * q[2]) - (q[1] * q[3])))

    t2 = +2.0 * (q[0] * q[2] - q[3] * q[1])
    t2 = np.where(t2>+1.0,+1.0,t2)
    t2 = np.where(t2<-1.0, -1.0, t2)
    Y = np.arcsin(t2)
    return np.arcsin(t2)

def rot(theta):

    pos = np.array( [[1,3], [4,2]] )
    tf = np.array([ [np.cos(theta),-np.sin(theta)], [np.sin(theta),np.cos(theta)] ])
    final = np.matmul(pos,tf)
    return final


q1=[0.707, 0, 0.707, 0] #90
q2=[1,0,1,0] # 180
#print(numpy_quat_to_theta(q1))
#print(numpy_quat_to_theta(q2))
print(rot(np.radians(30)))
