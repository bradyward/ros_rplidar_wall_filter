import rclpy
from rclpy.node import Node
#import rospy
from sensor_msgs.msg import LaserScan

from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        # Change 'topic' to match the topic to read from
        # For lidar, is 'sensor_msg/LaserScan'
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, message):
        #self.get_logger().info("Heard message: %s" % message.data)
        self.get_logger().info("Entire message: {0}\n\n\n\n".format(message))

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
