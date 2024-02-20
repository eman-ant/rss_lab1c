import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

class OpenSpacePublisher(Node):

    def __init__(self):
        super().__init__('open_space_publisher')
        self.subscription = self.create_subscription(
                LaserScan,
                'fake_scan',
                self.listener_callback,
                10)
        self.subscription

        self.publisher_distance_ = self.create_publisher(
                Float32,
                'open_space/distance',
                10)
        self.publisher_angle_ = self.create_publisher(
                Float32,
                'open_space/angle',
                10)


    def listener_callback(self, msg):
        ranges = msg.ranges
        max_range = Float32()
        max_angle = Float32() 

        max_range.data = max(ranges) 
        max_range_ix= ranges.index(max_range.data)
        
        max_angle.data = ((math.pi*4)/3)*(max_range_ix/400)-((math.pi*2)/3)
        self.publisher_distance_.publish(max_range)
        self.get_logger().info('Publishing longest range: %f' % max_range.data)
        
        self.publisher_angle_.publish(max_angle)
        self.get_logger().info('Publishing corresponding angle: %f' % max_angle.data)

def main(args=None):
    rclpy.init(args=args)

    open_space_publisher = OpenSpacePublisher()

    rclpy.spin(open_space_publisher)

    open_space_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
