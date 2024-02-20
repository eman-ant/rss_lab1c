import rclpy
import math
import random
from rclpy.node import Node
from sensor_msgs.msg import LaserScan 

class FakeScanPublisher(Node):

    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.publisher_ = self.create_publisher(
                LaserScan,
                'fake_scan',
                10)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
         msg = LaserScan()

         msg.angle_min = (-2/3)*math.pi
         msg.angle_max = (2/3)*math.pi
         msg.angle_increment = (1/300)*math.pi
         msg.scan_time = 15.0
         msg.range_min = 1.0
         msg.range_max = 10.0
         msg.ranges = [random.uniform(msg.angle_min, msg.angle_max) for _ in range(400)]

         msg.header.frame_id = 'base_link'
         msg.header.stamp = self.get_clock().now().to_msg()
            
         self.publisher_.publish(msg)
         self.get_logger().info('Publishing fake laser scan')

def main(args=None):
    rclpy.init(args=args)

    fake_scan_publisher = FakeScanPublisher()

    rclpy.spin(fake_scan_publisher)

    fake_scan_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
