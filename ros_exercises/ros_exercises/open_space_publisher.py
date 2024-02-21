import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import OpenSpace

class OpenSpacePublisher(Node):

    def __init__(self):
        super().__init__('open_space_publisher')
        
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('subscriber_topic', 'fake_scan'),
                    ('publisher_topic', 'open_space')
                ]
        )

        (param_subscriber_topic, param_publisher_topic) = self.get_parameters(['subscriber_topic', 'publisher_topic'])

        self.subscription = self.create_subscription(
                LaserScan,
                param_subscriber_topic.value,
                self.listener_callback,
                10)
        self.subscription

        self.publisher_ = self.create_publisher(
                OpenSpace,
                param_publisher_topic.value,
                10)

    def listener_callback(self, msg):
        ranges = msg.ranges
        max_range = Float32()
        max_angle = Float32() 
        open_space_msg = OpenSpace()

        max_range.data = max(ranges) 
        max_range_ix = ranges.index(max_range.data)
        
        max_angle.data = ((math.pi*4)/3)*(max_range_ix/400)-((math.pi*2)/3)
        
        open_space_msg.angle = max_angle
        open_space_msg.distance = max_range 

        self.publisher_.publish(open_space_msg)

        self.get_logger().info('Publishing longest distance: %f' % open_space_msg.distance.data)
        self.get_logger().info('Publishing corresponding angle: %f' % open_space_msg.angle.data)

def main(args=None):
    rclpy.init(args=args)

    open_space_publisher = OpenSpacePublisher()

    rclpy.spin(open_space_publisher)

    open_space_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
