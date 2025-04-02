import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String

import numpy as np
import math
import copy

import tf2_ros as tf2
#from tf2_ros import TransformException
#from tf2_ros.buffer import Buffer
#from tf2_ros.transform_listener import TransformListener

from example_interfaces.msg import String
from geometry_msgs.msg import PolygonStamped, Point32, Quaternion



class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        # Setup 2D lidar listener / publisher
        self.publisherTransformed = self.create_publisher(PolygonStamped, '/postRotation', 10)
        self.publisherTrimmed = self.create_publisher(LaserScan, '/postRotationLaser', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.subscription

        # Get bounding box
        #boxSubscriber = self.create_subscription(Point, '/arena', self.listener_callback, 10)

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
        self.min_x = -1# + self.r_pose[0]
        self.min_y = -1# + self.r_pose[1]
        self.max_x = 1 #+ self.r_pose[0]
        self.max_y = 1 #+ self.r_pose[1]


    ### For experimenting with point cloud
    def point_cloud_callback(self, message):
        # Get message data into a 2D numpy array, each row representing a point
        original_array = np.array(message.data)
        np_data = original_array.reshape(-1,32)

        # Remove half the points
        #new_data = np.array(np_data[::2].flatten()) #takes out half of the points

        """
        Convert the points to floats to make working with them possible
        Each x, and y must be converted from int int int int -> float
        First check if they are represented in little/big endian form
        Perform the conversion, likely using the struct package: https://stackoverflow.com/questions/57119566/converting-4-uint-8-values-to-float-in-python
        Ignore all other fields: y, intensity, ring, and time
        Perform bounds check on the x/y points in 2D. Write them back in their original form alongside the other fields
        Repeat for all other points

        Consider performance with this, whether its faster to:
            Convert all points, check all points, use mask to rewrite all valid points
            Check bounds and write as points are converted
        Second method should be significantly faster and easier as all original data is available and doesn't need to be converted back
        """
        if (message.isbigendian):
            1
        else:
            2

        # Update the message to match the new point array
        message.width = int(len(new_data) / 32)
        message.row_step = int(len(new_data))
        message.data = new_data

        # Publish
        self.pointCloudPublisher.publish(message)
    ###

    def print_no_data(self, m):
        tm = copy.deepcopy(m)
        tm.data = []
        self.get_logger().info("New message: {0}\n{1}\n\n".format(tm, len(m.data)))


    """
    Triggers when a LaserScan is recieved on /scan
    Trims all ranges outside a bounding box
    """
    def listener_callback(self, message):
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

        # Apply rotation
        transformed = self.apply_rotation(points_clean, self.quat_to_theta(r_pose.rotation))

        # Shift points according to robot's pose
        transformed = transformed + [r_pose.translation.x, r_pose.translation.y]

        # Cull any points out of bounds
        # TODO change self.min_x+r_pose.translation.x to adjust self.min_x at the start of getting the bound box so we don't have to add every time
        mask = ((self.min_x+r_pose.translation.x) < transformed[:,0]) & (transformed[:,0] < (self.max_x+r_pose.translation.x)) & ((self.min_y+r_pose.translation.y) < transformed[:,1]) & (transformed[:,1] < (self.max_y+r_pose.translation.y))
        culled_points = np.where(mask[:, np.newaxis], transformed, np.array([0,0]))

        # Construct the stamped polygon
        points_3d = np.hstack([culled_points, np.zeros((transformed.shape[0], 1), dtype=np.float32)]) # Tack 0 for z coord
        geo_message = PolygonStamped()
        geo_message.header.frame_id = "laser"
        geo_message.polygon.points = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points_3d]

        # Modify the original laser scan according to the map
        original_ranges = message.ranges
        culled_ranges = np.where(mask, original_ranges, self.default_bad).astype(np.float32)

        message.ranges = culled_ranges

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


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

