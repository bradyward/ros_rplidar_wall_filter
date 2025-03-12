import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import numpy as np
import math

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from example_interfaces.msg import String
from geometry_msgs.msg import PolygonStamped, Point32

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher = self.create_publisher(PolygonStamped, '/postRotation', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, message):
        self.get_logger().info("Entire message: {0}\n\n\n\n".format(message))

        # Ensure all points are float32 (required by Polygon) and clean out NaN/infinity values
        points = self.polar_to_cartesian(message.angle_min, message.ranges, message.angle_increment)
        points_clean = np.array(points, dtype=np.float32)
        points_clean = np.nan_to_num(points_clean, nan=0.0, posinf=0.0, neginf=0.0)


        # Apply rotation
        ###theta = self.quat_to_theta([0.9659258, 0, 0.258819, 0]) #30 degree rotation
        theta = self.quat_to_theta([0, 0, 0, 0]) #0 degree rotation
        transformed = self.apply_rotation(points_clean, theta)

        # Cull any points out of bounds
        min_x, min_y = 0.3, 0.3
        max_x, max_y = 2,2

        mask = (min_x < transformed[:,0]) & (transformed[:,0] < max_x) & (min_y < transformed[:,1]) & (transformed[:,1] < max_y)
        culled_points = np.where(mask[:, np.newaxis], transformed, np.array([0,0]))

        # Add a 0 for the z coordinate (required to publish polygons)
        points_3d = np.hstack([culled_points, np.zeros((transformed.shape[0], 1), dtype=np.float32)]) 

        # Construct the stamped polygon
        geo_message = PolygonStamped()
        geo_message.header.frame_id = "laser"
        geo_message.polygon.points = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points_3d]

        self.publisher.publish(geo_message)

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
        t2 = 2.0 * (quat[0] * quat[2] - quat[3] * quat[1])
        #t2 = 2.0 * (quat.w * quat.y - quat.z * quat.x)
        t2 = np.where(t2>+1.0,+1.0,t2)
        t2 = np.where(t2<-1.0, -1.0, t2)
        return np.arcsin(t2)

    """
    Creates and multiplies a transformation matrix over all points
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

