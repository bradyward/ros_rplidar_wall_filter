import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String

import numpy as np
import math
import copy
import struct

import tf2_ros as tf2
#from tf2_ros import TransformException
#from tf2_ros.buffer import Buffer
#from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PolygonStamped, Point32, Quaternion#, Rectangle

class WallFilterNode(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        # Setup 2D lidar listener / publisher
        self.publisherTransformed = self.create_publisher(PolygonStamped, '/postRotation', 10)
        self.publisherTrimmed = self.create_publisher(LaserScan, '/postRotationLaser', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laserScan_callback, 10)
        self.subscription

        # Get bounding box
        # Import from kurome/msg in main repo
        #self.arenaSubscriber = self.create_subscription(Rectangle, '/kurome/arena', self.listener_callback, 10)
        self.min_x = -1# + self.r_pose[0]
        self.min_y = -1# + self.r_pose[1]
        self.max_x = 3 #+ self.r_pose[0]
        self.max_y = 0 #+ self.r_pose[1]

        # Setup 3D lidar listener / publisher
        self.pointCloudSubscriber = self.create_subscription(PointCloud2, '/unilidar/cloud', self.point_cloud_callback, 10)
        self.pointCloudPublisher = self.create_publisher(PointCloud2, '/testPointCloud', 100)
        
        # Setup transform listener
        self.tf_buffer = tf2.Buffer()
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self)

        ### Parameters ###
        #self.target_frame = self.declare_parameter('target_frame', 'default_value').get_parameter_value().string_value

        # Should get these from tf listener during callback. Parameters just for testing
        #self.r_pose = [-.5,-.5] 
        #self.quat = [0,0,0,0] # 0 degree rotation
        #self.quat = [0.9659258, 0, 0.258819, 0] # 30 degree rotation

        self.default_bad = 0.0

    def point_cloud_callback(self, message):
        datatype_sizes = {
            1: 1,  # INT8
            2: 1,  # UINT8
            3: 2,  # INT16
            4: 2,  # UINT16
            5: 4,  # INT32
            6: 4,  # UINT32
            7: 4,  # FLOAT32
            8: 8   # FLOAT64
        }

        # Get transform message
        try:
            r_pose = self.tf_buffer.lookup_transform("target_frame", "source_frame", rclpy.time.Time()).transform
            #self.get_logger().info(f'Transform quat: {self.quat}')
        except Exception as ex:
            self.get_logger().info(f'Could not transform : {ex}')
            return

		# Get bounding box relative to lidar device
        box = [ [self.min_x, self.min_y], [self.max_x, self.max_y] ]
        box = self.transform_bounding_box(box, self.quat_to_theta(r_pose.rotation), r_pose.translation)
        
        # Get size of x and y
        x_size = datatype_sizes[message.fields[0].datatype]
        y_size = datatype_sizes[message.fields[1].datatype]
        x = message.fields[0]
        y = message.fields[1]
        
        #valid_points = np.array([]).astype(np.uint8)
        valid_points = []
        
        for i in range(0, message.row_step*message.height, message.point_step):
        	# Create formatter for unpacking ints to float
        	formatter = '>f' if message.is_bigendian else '<f'
        
        	# Get the actual x and y coordinate
        	x_point = struct.unpack(formatter, bytes(message.data[i+x.offset : i+x.offset + x_size]))[0]
        	y_point = struct.unpack(formatter, bytes(message.data[i+y.offset : i+y.offset + y_size]))[0]
        
        	# Check if the point is within bounds
        	if box[0][0] <= x_point <= box[1][0] and box[0][1] <= y_point <= box[1][1]:
        		# Store all data associated with this point
        		valid_points.extend(message.data[i:i+message.point_step])
        
        # Merge new point array into original message and publish
        # Adjust total size of message width
        message.data = valid_points
        message.row_step = int(len(valid_points)) # Might need to force as uint32
        message.width = int(message.row_step / message.point_step) # Might need to force as uint32
        self.pointCloudPublisher.publish(message)

    """
    Triggers when a LaserScan is recieved on /scan
    Trims all ranges outside a bounding box
    """
    def laserScan_callback(self, message):
        #self.get_logger().info("Entire message: {0}\n".format(message))

        # Get transform message
        try:
            r_pose = self.tf_buffer.lookup_transform("target_frame", "source_frame", rclpy.time.Time()).transform
            #self.get_logger().info(f'Transform quat: {self.quat}')
        except Exception as ex:
            self.get_logger().info(f'Could not transform : {ex}')
            return

        # Ensure all points are float32 (required by Polygon) and clean out NaN/infinity values
        points = self.polar_to_cartesian(message.angle_min, message.ranges, message.angle_increment)
        points = np.array(points, dtype=np.float32)
        points_clean = np.nan_to_num(points, nan=self.default_bad, posinf=self.default_bad, neginf=self.default_bad)

        ### TODO remove these two transformations and adjust bounding box instead
        # Apply rotation then shift points according to robot's pose
        transformed = self.apply_rotation(points_clean, self.quat_to_theta(r_pose.rotation))
        transformed = transformed + [r_pose.translation.x, r_pose.translation.y]
        #box = [ [self.min_x, self.min_y], [self.max_x, self.max_y] ]
        #transformed_box = self.transform_bounding_box(box, self.quat_to_theta(r_pose.rotation), r_pose.translation)

        # Cull any points out of bounds
        mask = ((self.min_x+r_pose.translation.x) < transformed[:,0]) & (transformed[:,0] < (self.max_x+r_pose.translation.x)) & ((self.min_y+r_pose.translation.y) < transformed[:,1]) & (transformed[:,1] < (self.max_y+r_pose.translation.y))
        #mask = ( (transformed_box[0][0] < points_clean[:,0]) & (points_clean[:,0] < transformed_box[1][0]) ) & ( (transformed_box[0][1] < points_clean[:,1]) & (points_clean[:,1] < transformed_box[1][1]) )
        culled_points = np.where(mask[:, np.newaxis], transformed, np.array([0,0]))

        # Modify the original laser scan according to the map
        message.ranges = np.where(mask, message.ranges, self.default_bad).astype(np.float32)

        # Publish
        self.publisherTransformed.publish(geo_message)
        self.publisherTrimmed.publish(message)

    """
    Takes in a lidar's LaserScan message and converts it to cartesian coordinates
    """
    def polar_to_cartesian(self, start_angle, distances, angle_step):
        angles = start_angle + np.arange(len(distances)) * angle_step
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        return np.column_stack((x, y))
    
    """
    Convert a quaternion to its Yaw angle (theta) so it is usable in a transformation matrix
    """
    def quat_to_theta(self, quat):
        # Assume quaternions are in form here: https://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Quaternion.html
        #t2 = 2.0 * (quat[0] * quat[2] - quat[3] * quat[1])
        t2 = 2.0 * (quat.w * quat.y - quat.z * quat.x)
        t2 = np.where(t2>+1.0,+1.0,t2)
        t2 = np.where(t2<-1.0, -1.0, t2)
        return np.arcsin(t2)

    """
    Multiplies a transformation matrix over all points
    """
    def apply_rotation(self, points, theta):
        tf_matrix = np.array([ [np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)] ])
        after_rotation = np.matmul(points,tf_matrix)
        return after_rotation

    """
    Transform the box to the lidar's coordinate frame
    param box should be a 2d array
    """
    def transform_bounding_box(self, box, theta, current_pose):
    	# Corners of the global bounding box
        corners = np.array( [
			[self.min_x, self.min_y],
        	[self.min_x, self.max_y],
        	[self.max_x, self.min_y],
        	[self.max_x, self.max_y]
            ]
		)

    	# Step 1: Translate by -current_pose
        translated_points = corners - np.array([current_pose.x + of, current_pose.y + of])

    	# Step 2: Inverse rotate by -theta
        transformation_matrix = np.array([
            [np.cos(-theta), -np.sin(-theta)],
            [np.sin(-theta), np.cos(-theta)]
            ])
        transformed = np.dot(translated_points, transformation_matrix.T)

		# Return result as [ [min_x, min_y], [max_x, max_y] ]
        new_min = np.min(transformed, axis=0).tolist()
        new_max = np.max(transformed, axis=0).tolist()
        return [ [new_min[0], new_min[1]], [new_max[0], new_max[1]] ]

def main(args=None):
    rclpy.init(args=args)
    wall_filter = WallFilterNode()
    rclpy.spin(wall_filter)
    wall_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

